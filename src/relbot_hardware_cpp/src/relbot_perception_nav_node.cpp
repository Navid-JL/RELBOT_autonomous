#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "relbot_hardware_cpp/relbot_navigation_ffi.h"
#include "relbot_interfaces/msg/arm_command.hpp"
#include "relbot_interfaces/msg/detected_object.hpp"
#include "relbot_interfaces/msg/manipulator_state.hpp"
#include "relbot_interfaces/msg/mission_status.hpp"

namespace relbot::nav {

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

bool sendAll(int fd, const void* data, size_t len) {
  const auto* ptr = static_cast<const uint8_t*>(data);
  size_t sent = 0;
  while (sent < len) {
    const ssize_t n = ::send(fd, ptr + sent, len - sent, MSG_NOSIGNAL);
    if (n <= 0) {
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

class MjpegPreviewServer {
 public:
  using CommandCallback = std::function<std::string(const std::string&)>;

  explicit MjpegPreviewServer(int port, CommandCallback cmd_cb = nullptr)
      : port_(port), cmd_cb_(std::move(cmd_cb)) {}
  ~MjpegPreviewServer() { stop(); }

  bool start() {
    if (port_ <= 0) {
      return false;
    }
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
      return false;
    }
    int opt = 1;
    ::setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      ::close(server_fd_);
      server_fd_ = -1;
      return false;
    }
    if (::listen(server_fd_, 4) < 0) {
      ::close(server_fd_);
      server_fd_ = -1;
      return false;
    }

    running_.store(true);
    accept_thread_ = std::thread([this]() { acceptLoop(); });
    return true;
  }

  void stop() {
    running_.store(false);
    if (server_fd_ >= 0) {
      ::shutdown(server_fd_, SHUT_RDWR);
      ::close(server_fd_);
      server_fd_ = -1;
    }
    if (accept_thread_.joinable()) {
      accept_thread_.join();
    }
  }

  void updateFrame(const std::vector<uint8_t>& jpeg_bytes) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_jpeg_ = jpeg_bytes;
    ++frame_seq_;
  }

 private:
  void acceptLoop() {
    while (running_.load()) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      int client_fd = ::accept(server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
      if (client_fd < 0) {
        if (!running_.load()) {
          break;
        }
        continue;
      }
      std::thread(&MjpegPreviewServer::handleClient, this, client_fd).detach();
    }
  }

  void handleClient(int client_fd) {
    char req_buf[1024]{};
    const ssize_t n = ::recv(client_fd, req_buf, sizeof(req_buf) - 1, 0);
    if (n <= 0) {
      ::close(client_fd);
      return;
    }
    const std::string req(req_buf, static_cast<size_t>(n));

    if (req.find("GET /stream") == 0) {
      const std::string header =
          "HTTP/1.1 200 OK\r\n"
          "Cache-Control: no-cache\r\n"
          "Pragma: no-cache\r\n"
          "Connection: close\r\n"
          "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
      if (!sendAll(client_fd, header.data(), header.size())) {
        ::close(client_fd);
        return;
      }

      uint64_t last_seq = 0;
      while (running_.load()) {
        std::vector<uint8_t> jpg;
        {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          if (frame_seq_ != last_seq && !latest_jpeg_.empty()) {
            jpg = latest_jpeg_;
            last_seq = frame_seq_;
          }
        }
        if (jpg.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
          continue;
        }
        const std::string part =
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
            std::to_string(jpg.size()) + "\r\n\r\n";
        if (!sendAll(client_fd, part.data(), part.size()) ||
            !sendAll(client_fd, jpg.data(), jpg.size()) ||
            !sendAll(client_fd, "\r\n", 2)) {
          break;
        }
      }
    } else if (req.find("GET /snapshot.jpg") == 0) {
      std::vector<uint8_t> jpg;
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        jpg = latest_jpeg_;
      }
      if (jpg.empty()) {
        const std::string resp = "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\n\r\n";
        sendAll(client_fd, resp.data(), resp.size());
      } else {
        const std::string resp =
            "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nCache-Control: no-cache\r\n"
            "Content-Length: " +
            std::to_string(jpg.size()) + "\r\n\r\n";
        sendAll(client_fd, resp.data(), resp.size());
        sendAll(client_fd, jpg.data(), jpg.size());
      }
    } else if (req.find("GET /api/cmd") == 0) {
      std::string body = "{\"ok\":false}";
      if (cmd_cb_) {
        const auto qpos = req.find('?');
        const auto sp = req.find(' ', 4);
        const std::string query =
            (qpos != std::string::npos && sp != std::string::npos && qpos < sp)
                ? req.substr(qpos + 1, sp - qpos - 1)
                : "";
        body = cmd_cb_(query);
      }
      const std::string resp =
          "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nCache-Control: no-cache\r\n"
          "Connection: close\r\nContent-Length: " +
          std::to_string(body.size()) + "\r\n\r\n" + body;
      sendAll(client_fd, resp.data(), resp.size());
    } else {
      const std::string html =
          "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
          "<title>RELbot Rust Object Detection & Teleop</title>"
          "<style>body{background:#111;color:#eee;font-family:monospace;text-align:center;margin:16px;}"
          "img{border:2px solid #4caf50;max-width:95%;width:520px;height:auto;border-radius:6px;}"
          ".btns{margin:12px auto;display:inline-grid;grid-template-columns:repeat(3,90px);gap:8px;}"
          "button{background:#222;color:#eee;border:1px solid #555;padding:12px;font-family:monospace;"
          "font-size:14px;border-radius:6px;cursor:pointer;user-select:none;}"
          "button:active,button.active{background:#4caf50;color:#000;font-weight:bold;}"
          ".mode-bar{margin:10px;}"
          "</style></head><body>"
          "<h2>RELbot Live Rust Object Detection & Control</h2>"
          "<img src=\"/stream\" alt=\"RELbot live stream\"/>"
          "<div class=\"mode-bar\">"
          "<button id=\"btnAuto\" onclick=\"setMode('auto')\">AUTO (Follow Ball)</button> "
          "<button id=\"btnMan\" onclick=\"setMode('manual')\">MANUAL (WASD / Buttons)</button>"
          "<div id=\"status\" style=\"margin-top:6px;color:#8bc34a;\">Hold WASD / Arrow keys or buttons in MANUAL mode</div>"
          "</div>"
          "<div class=\"btns\">"
          "<div></div><button onmousedown=\"drv(0.18,0)\" onmouseup=\"drv(0,0)\" ontouchstart=\"drv(0.18,0)\" ontouchend=\"drv(0,0)\">W / ▲</button><div></div>"
          "<button onmousedown=\"drv(0,0.65)\" onmouseup=\"drv(0,0)\" ontouchstart=\"drv(0,0.65)\" ontouchend=\"drv(0,0)\">A / ◀</button>"
          "<button onclick=\"drv(0,0)\" style=\"background:#b71c1c;\">STOP</button>"
          "<button onmousedown=\"drv(0,-0.65)\" onmouseup=\"drv(0,0)\" ontouchstart=\"drv(0,-0.65)\" ontouchend=\"drv(0,0)\">D / ▶</button>"
          "<div></div><button onmousedown=\"drv(-0.15,0)\" onmouseup=\"drv(0,0)\" ontouchstart=\"drv(-0.15,0)\" ontouchend=\"drv(0,0)\">S / ▼</button><div></div>"
          "</div>"
          "<script>"
          "let tmr=null;"
          "function api(q){fetch('/api/cmd?'+q).then(r=>r.json()).then(d=>{"
          "document.getElementById('status').innerText='Mode: '+(d.manual?'MANUAL':'AUTO')+' | v='+d.v.toFixed(2)+' w='+d.w.toFixed(2);"
          "}).catch(()=>{});}"
          "function setMode(m){api('mode='+m);}"
          "function drv(v,w){if(tmr){clearInterval(tmr);tmr=null;}api('mode=manual&v='+v+'&w='+w);"
          "if(v!==0||w!==0){tmr=setInterval(()=>api('mode=manual&v='+v+'&w='+w),250);}}"
          "const keys={};"
          "function updKey(){"
          "let v=0,w=0;"
          "if(keys['KeyW']||keys['ArrowUp'])v+=0.18;"
          "if(keys['KeyS']||keys['ArrowDown'])v-=0.15;"
          "if(keys['KeyA']||keys['ArrowLeft'])w+=0.65;"
          "if(keys['KeyD']||keys['ArrowRight'])w-=0.65;"
          "drv(v,w);}"
          "window.addEventListener('keydown',e=>{if(e.repeat)return;"
          "if(e.code==='Space'){Object.keys(keys).forEach(k=>delete keys[k]);drv(0,0);e.preventDefault();return;}"
          "if(e.code==='KeyM'){setMode('toggle');return;}"
          "if(['KeyW','KeyA','KeyS','KeyD','ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.code)){"
          "keys[e.code]=true;updKey();e.preventDefault();}});"
          "window.addEventListener('keyup',e=>{"
          "if(keys[e.code]){delete keys[e.code];updKey();e.preventDefault();}});"
          "</script></body></html>";
      const std::string resp =
          "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n"
          "Content-Length: " +
          std::to_string(html.size()) + "\r\n\r\n" + html;
      sendAll(client_fd, resp.data(), resp.size());
    }

    ::close(client_fd);
  }

  int port_;
  CommandCallback cmd_cb_;
  int server_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread accept_thread_;
  std::mutex frame_mutex_;
  std::vector<uint8_t> latest_jpeg_;
  uint64_t frame_seq_{0};
};

}  // namespace

class RelbotPerceptionNavNode : public rclcpp::Node {
 public:
  RelbotPerceptionNavNode() : Node("relbot_perception_nav_node") {
    camera_source_ = this->declare_parameter<std::string>("camera_source", "v4l2");
    camera_device_ = this->declare_parameter<int>("camera_device", 0);
    image_topic_ = this->declare_parameter<std::string>("image_topic", "/image");
    capture_width_ = this->declare_parameter<int>("capture_width", 640);
    capture_height_ = this->declare_parameter<int>("capture_height", 480);
    use_mjpg_ = this->declare_parameter<bool>("use_mjpg", false);
    target_width_ = this->declare_parameter<int>("image_width", 320);
    target_height_ = this->declare_parameter<int>("image_height", 240);
    rotate_180_ = this->declare_parameter<bool>("rotate_180", true);
    fps_ = this->declare_parameter<double>("fps", 20.0);
    enable_motion_ = this->declare_parameter<bool>("enable_motion", false);
    follow_only_ = this->declare_parameter<bool>("follow_only", false);
    manual_mode_ = this->declare_parameter<bool>("manual_mode", false);
    search_when_lost_ = this->declare_parameter<bool>("search_when_lost", false);
    reacquire_timeout_sec_ = this->declare_parameter<double>("reacquire_timeout_sec", 1.5);
    min_ball_confidence_ = this->declare_parameter<double>("min_ball_confidence", 0.42);
    follow_standoff_m_ = this->declare_parameter<double>("follow_standoff_m", 0.25);
    http_port_ = this->declare_parameter<int>("http_port", 8080);
    snapshot_path_ = this->declare_parameter<std::string>("snapshot_path", "/tmp/relbot_debug.jpg");

    // Configure Rust perception parameters (tuned for real RELbot green ball + blue delivery marker)
    RelbotPerceptionConfigC pcfg = relbot_perception_config_default();
    pcfg.image_width = static_cast<uint32_t>(target_width_);
    pcfg.image_height = static_cast<uint32_t>(target_height_);
    pcfg.horizontal_fov_rad = this->declare_parameter<double>("horizontal_fov_rad", pcfg.horizontal_fov_rad);
    pcfg.vertical_fov_rad = this->declare_parameter<double>("vertical_fov_rad", pcfg.vertical_fov_rad);
    pcfg.pickup_object_height_m = this->declare_parameter<double>("pickup_object_height_m", 0.065);
    pcfg.delivery_marker_height_m = this->declare_parameter<double>("delivery_marker_height_m", 0.120);
    pcfg.min_blob_pixels = static_cast<uint32_t>(this->declare_parameter<int>("min_blob_pixels", 45));

    pcfg.pickup_hsv.h_min = static_cast<float>(this->declare_parameter<double>("pickup_h_min", 75.0));
    pcfg.pickup_hsv.h_max = static_cast<float>(this->declare_parameter<double>("pickup_h_max", 165.0));
    pcfg.pickup_hsv.s_min = static_cast<float>(this->declare_parameter<double>("pickup_s_min", 0.35));
    pcfg.pickup_hsv.v_min = static_cast<float>(this->declare_parameter<double>("pickup_v_min", 0.25));

    pcfg.delivery_hsv.h_min = static_cast<float>(this->declare_parameter<double>("delivery_h_min", 200.0));
    pcfg.delivery_hsv.h_max = static_cast<float>(this->declare_parameter<double>("delivery_h_max", 255.0));
    pcfg.delivery_hsv.s_min = static_cast<float>(this->declare_parameter<double>("delivery_s_min", 0.55));
    pcfg.delivery_hsv.v_min = static_cast<float>(this->declare_parameter<double>("delivery_v_min", 0.40));

    detector_ = relbot_detector_create(&pcfg);

    // Configure Rust navigation & FSM parameters
    nav_cfg_ = relbot_navigation_config_default();
    nav_cfg_.pickup_standoff_m = this->declare_parameter<double>("pickup_standoff_m", nav_cfg_.pickup_standoff_m);
    nav_cfg_.delivery_standoff_m = this->declare_parameter<double>("delivery_standoff_m", nav_cfg_.delivery_standoff_m);
    nav_cfg_.kp_linear = this->declare_parameter<double>("kp_linear", 0.50);
    nav_cfg_.kp_angular = this->declare_parameter<double>("kp_angular", 1.40);
    nav_cfg_.max_linear_speed_m_s = this->declare_parameter<double>("max_linear_speed_m_s", 0.18);
    nav_cfg_.max_angular_speed_rad_s = this->declare_parameter<double>("max_angular_speed_rad_s", 0.80);
    nav_cfg_.search_spin_rad_s = this->declare_parameter<double>("search_spin_rad_s", 0.35);

    mission_ = relbot_mission_create(&nav_cfg_);

    // Publishers
    detected_pub_ = this->create_publisher<relbot_interfaces::msg::DetectedObject>(
        "/perception/detected_object", 10);
    debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/perception/debug_image", 10);
    mission_status_pub_ = this->create_publisher<relbot_interfaces::msg::MissionStatus>(
        "/mission/status", 10);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    arm_cmd_pub_ = this->create_publisher<relbot_interfaces::msg::ArmCommand>("/arm/command", 10);

    // Subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          latest_odom_.x_m = msg->pose.pose.position.x;
          latest_odom_.y_m = msg->pose.pose.position.y;
          const auto& q = msg->pose.pose.orientation;
          latest_odom_.yaw_rad = std::atan2(
              2.0 * (q.w * q.z + q.x * q.y),
              1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        });

    manip_state_sub_ = this->create_subscription<relbot_interfaces::msg::ManipulatorState>(
        "/arm/state", 10,
        [this](const relbot_interfaces::msg::ManipulatorState::SharedPtr msg) {
          latest_manip_.current_arm_angle_rad = msg->current_arm_angle_rad;
          latest_manip_.current_arm_velocity_rad_s = msg->current_arm_velocity_rad_s;
          latest_manip_.arm_load_ma = msg->arm_load;
          latest_manip_.current_gripper_angle_deg = msg->current_gripper_angle_deg;
          latest_manip_.target_reached = msg->target_reached;
          latest_manip_.object_grasped = msg->object_grasped;
          latest_manip_.error_code = msg->error_code;
        });

    // Default safe manipulator state before hardware node publishes
    latest_manip_.current_arm_angle_rad = 0.10;
    latest_manip_.current_gripper_angle_deg = 20.0;
    latest_manip_.target_reached = true;
    latest_manip_.object_grasped = false;
    latest_manip_.error_code = 0;

    manual_cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/manual/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(manual_mutex_);
          if (msg->linear.z < -0.5) {
            manual_mode_ = false;
            manual_linear_x_ = 0.0;
            manual_angular_z_ = 0.0;
          } else if (msg->linear.z > 0.5) {
            manual_mode_ = true;
          }
          manual_linear_x_ = msg->linear.x;
          manual_angular_z_ = msg->angular.z;
          last_manual_cmd_time_ = this->now();
          has_manual_cmd_ = true;
        });

    if (http_port_ > 0) {
      preview_server_ = std::make_unique<MjpegPreviewServer>(
          http_port_, [this](const std::string& query) { return handleHttpCommand(query); });
      if (preview_server_->start()) {
        RCLCPP_INFO(this->get_logger(),
                    "Live MJPEG preview available at http://<relbot-ip>:%d/ (and /snapshot.jpg)",
                    http_port_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Could not start HTTP preview on port %d", http_port_);
      }
    }

    last_step_time_ = this->now();

    if (camera_source_ == "v4l2") {
      cap_.open(camera_device_, cv::CAP_V4L2);
      if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to open V4L2 camera /dev/video%d! Check camera connection.",
                     camera_device_);
      } else {
        if (use_mjpg_) {
          cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        }
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, capture_width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, capture_height_);
        cap_.set(cv::CAP_PROP_FPS, 30.0);
        const int actual_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        const int actual_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
        RCLCPP_INFO(this->get_logger(),
                    "Opened /dev/video%d capture=%dx%d (mjpg=%s) -> proc=%dx%d (rotate_180=%s, enable_motion=%s)",
                    camera_device_, actual_w, actual_h, use_mjpg_ ? "true" : "false",
                    target_width_, target_height_,
                    rotate_180_ ? "true" : "false",
                    enable_motion_ ? "true" : "false");
      }
      const auto period_ms = static_cast<int>(std::max(10.0, 1000.0 / std::max(1.0, fps_)));
      capture_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(period_ms),
          std::bind(&RelbotPerceptionNavNode::v4l2TimerCallback, this));
    } else {
      image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
          image_topic_, rclcpp::SensorDataQoS(),
          std::bind(&RelbotPerceptionNavNode::rosImageCallback, this, std::placeholders::_1));
      RCLCPP_INFO(this->get_logger(),
                  "Subscribed to ROS image topic %s (rotate_180=%s, enable_motion=%s)",
                  image_topic_.c_str(), rotate_180_ ? "true" : "false",
                  enable_motion_ ? "true" : "false");
    }
  }

  ~RelbotPerceptionNavNode() override {
    if (preview_server_) {
      preview_server_->stop();
    }
    if (cap_.isOpened()) {
      cap_.release();
    }
    relbot_detector_destroy(detector_);
    relbot_mission_destroy(mission_);
  }

 private:
  void v4l2TimerCallback() {
    if (!cap_.isOpened()) {
      return;
    }
    cv::Mat frame_bgr;
    if (!cap_.read(frame_bgr) || frame_bgr.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Failed to capture frame from /dev/video%d", camera_device_);
      return;
    }
    processBgrFrame(frame_bgr);
  }

  void rosImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      return;
    }
    cv::Mat frame_bgr;
    if (msg->encoding == "bgr8") {
      frame_bgr = cv::Mat(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
                          const_cast<uint8_t*>(msg->data.data()), msg->step)
                      .clone();
    } else if (msg->encoding == "rgb8") {
      cv::Mat rgb(static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
                  const_cast<uint8_t*>(msg->data.data()), msg->step);
      cv::cvtColor(rgb, frame_bgr, cv::COLOR_RGB2BGR);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Unsupported image encoding '%s' (expected bgr8 or rgb8)",
                           msg->encoding.c_str());
      return;
    }
    processBgrFrame(frame_bgr);
  }

  void drawDetectionBox(cv::Mat& img, const RelbotDetectedObjectC& det,
                        const cv::Scalar& color, const std::string& label) {
    if (!det.valid) {
      return;
    }
    const int w = img.cols;
    const int h = img.rows;
    const int cx = static_cast<int>((det.bbox_center_x + 1.0f) * 0.5f * w);
    const int cy = static_cast<int>((det.bbox_center_y + 1.0f) * 0.5f * h);
    const int bw = static_cast<int>(det.bbox_width * w);
    const int bh = static_cast<int>(det.bbox_height * h);

    const int x0 = std::max(0, cx - bw / 2);
    const int y0 = std::max(0, cy - bh / 2);
    const int x1 = std::min(w - 1, cx + bw / 2);
    const int y1 = std::min(h - 1, cy + bh / 2);

    cv::rectangle(img, cv::Point(x0, y0), cv::Point(x1, y1), color, 2);
    cv::drawMarker(img, cv::Point(cx, cy), color, cv::MARKER_CROSS, 12, 2);

    char info[128];
    std::snprintf(info, sizeof(info), "%s d=%.2fm b=%.1fdeg c=%.2f",
                  label.c_str(), det.estimated_distance_m,
                  det.estimated_bearing_rad * kRadToDeg, det.confidence);
    cv::putText(img, info, cv::Point(std::max(4, x0), std::max(16, y0 - 6)),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv::LINE_AA);
  }

  std::string handleHttpCommand(const std::string& query) {
    std::lock_guard<std::mutex> lock(manual_mutex_);
    auto getParam = [&](const std::string& key) -> std::string {
      const std::string prefix = key + "=";
      auto pos = query.find(prefix);
      if (pos == std::string::npos) {
        return "";
      }
      pos += prefix.size();
      auto end = query.find('&', pos);
      return (end == std::string::npos) ? query.substr(pos) : query.substr(pos, end - pos);
    };

    const std::string mode = getParam("mode");
    if (mode == "manual") {
      manual_mode_ = true;
    } else if (mode == "auto") {
      manual_mode_ = false;
      manual_linear_x_ = 0.0;
      manual_angular_z_ = 0.0;
    } else if (mode == "toggle") {
      manual_mode_ = !manual_mode_;
      manual_linear_x_ = 0.0;
      manual_angular_z_ = 0.0;
    }

    const std::string v_str = getParam("v");
    const std::string w_str = getParam("w");
    if (!v_str.empty()) {
      manual_linear_x_ = std::clamp(std::atof(v_str.c_str()), -0.22, 0.22);
      last_manual_cmd_time_ = this->now();
      has_manual_cmd_ = true;
    }
    if (!w_str.empty()) {
      manual_angular_z_ = std::clamp(std::atof(w_str.c_str()), -1.0, 1.0);
      last_manual_cmd_time_ = this->now();
      has_manual_cmd_ = true;
    }

    char buf[128];
    std::snprintf(buf, sizeof(buf), "{\"ok\":true,\"manual\":%s,\"v\":%.3f,\"w\":%.3f}",
                  manual_mode_ ? "true" : "false", manual_linear_x_, manual_angular_z_);
    return std::string(buf);
  }

  void publishDetectedObject(const RelbotDetectedObjectC& det, const std::string& label,
                             const rclcpp::Time& stamp) {
    relbot_interfaces::msg::DetectedObject msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "camera_link";
    msg.label = label;
    msg.confidence = det.confidence;
    msg.bbox_center_x = det.bbox_center_x;
    msg.bbox_center_y = det.bbox_center_y;
    msg.bbox_width = det.bbox_width;
    msg.bbox_height = det.bbox_height;
    msg.estimated_distance_m = det.estimated_distance_m;
    msg.estimated_bearing_rad = det.estimated_bearing_rad;
    detected_pub_->publish(msg);
  }

  void processBgrFrame(cv::Mat frame_bgr) {
    if (rotate_180_) {
      cv::rotate(frame_bgr, frame_bgr, cv::ROTATE_180);
    }
    if (frame_bgr.cols != target_width_ || frame_bgr.rows != target_height_) {
      cv::resize(frame_bgr, frame_bgr, cv::Size(target_width_, target_height_));
    }

    cv::Mat frame_rgb;
    cv::cvtColor(frame_bgr, frame_rgb, cv::COLOR_BGR2RGB);
    if (!frame_rgb.isContinuous()) {
      frame_rgb = frame_rgb.clone();
    }

    const size_t rgb_bytes = frame_rgb.total() * frame_rgb.elemSize();
    auto pickup_det = relbot_detector_detect_rgb8(
        detector_, frame_rgb.data, rgb_bytes,
        static_cast<uint32_t>(frame_rgb.cols), static_cast<uint32_t>(frame_rgb.rows), 0);
    // Reject low-fill-ratio noise blobs (e.g. thin wires or distant reflections)
    if (pickup_det.valid &&
        (pickup_det.confidence < static_cast<float>(min_ball_confidence_) ||
         pickup_det.estimated_distance_m > 2.5)) {
      pickup_det.valid = false;
    }
    RelbotDetectedObjectC delivery_det{};
    if (!follow_only_) {
      delivery_det = relbot_detector_detect_rgb8(
          detector_, frame_rgb.data, rgb_bytes,
          static_cast<uint32_t>(frame_rgb.cols), static_cast<uint32_t>(frame_rgb.rows), 1);
    }

    const rclcpp::Time now = this->now();
    double dt = (now - last_step_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      dt = 0.05;
    }
    last_step_time_ = now;

    RelbotMissionOutputC mission_out{};
    const char* state_name = "IDLE";

    ++frame_count_;
    if (pickup_det.valid) {
      ++consecutive_ball_frames_;
    } else {
      consecutive_ball_frames_ = 0;
    }

    // Wait for camera auto-exposure warmup (first 15 frames) and 2-frame debounce
    const bool warmed_up = frame_count_ > 15;
    const bool stable_ball = warmed_up && (consecutive_ball_frames_ >= 2);

    bool is_manual = false;
    double man_v = 0.0;
    double man_w = 0.0;
    {
      std::lock_guard<std::mutex> lock(manual_mutex_);
      is_manual = manual_mode_;
      if (has_manual_cmd_ && (now - last_manual_cmd_time_).seconds() <= 0.8) {
        man_v = manual_linear_x_;
        man_w = manual_angular_z_;
      }
    }

    if (is_manual) {
      mission_out.status_code = 0;
      mission_out.cmd_linear_x_m_s = man_v;
      mission_out.cmd_angular_z_rad_s = man_w;
      mission_out.distance_to_goal_m = pickup_det.valid ? pickup_det.estimated_distance_m : 0.0;
      state_name = "MANUAL_CTRL";
    } else if (follow_only_) {
      if (!warmed_up) {
        mission_out.status_code = 0;
        state_name = "CAM_WARMUP";
      } else if (stable_ball) {
        has_seen_ball_ = true;
        last_seen_bearing_rad_ = pickup_det.estimated_bearing_rad;
        last_seen_time_ = now;
        const auto tw = relbot_controller_follow_target(
            &nav_cfg_, &pickup_det, follow_standoff_m_, 0.03, 0.07);
        mission_out.status_code = 2;
        mission_out.cmd_linear_x_m_s = tw.linear_x_m_s;
        mission_out.cmd_angular_z_rad_s = tw.angular_z_rad_s;
        mission_out.distance_to_goal_m = pickup_det.estimated_distance_m;
        state_name = (tw.linear_x_m_s == 0.0 && tw.angular_z_rad_s == 0.0)
                         ? "FOLLOW_HOLD"
                         : "FOLLOW_BALL";
      } else if (search_when_lost_) {
        const double spin_dir = (has_seen_ball_ && last_seen_bearing_rad_ < 0.0) ? -1.0 : 1.0;
        mission_out.status_code = 1;
        mission_out.cmd_angular_z_rad_s = spin_dir * nav_cfg_.search_spin_rad_s;
        state_name = "SEARCH_BALL";
      } else {
        mission_out.status_code = 1;
        state_name = "WAIT_BALL";
      }
    } else {
      mission_out = relbot_mission_step(
          mission_, &latest_odom_,
          stable_ball ? &pickup_det : nullptr,
          (warmed_up && delivery_det.valid) ? &delivery_det : nullptr,
          &latest_manip_, dt);
      state_name = relbot_mission_state_name(mission_out.state_raw);
    }

    if (pickup_det.valid) {
      publishDetectedObject(pickup_det, "pickup_target", now);
    }
    if (delivery_det.valid) {
      publishDetectedObject(delivery_det, "delivery_zone", now);
    }

    relbot_interfaces::msg::MissionStatus status_msg;
    status_msg.header.stamp = now;
    status_msg.state = mission_out.status_code;
    status_msg.state_name = state_name;
    status_msg.distance_to_goal_m = mission_out.distance_to_goal_m;
    status_msg.holding_object = mission_out.holding_object;
    mission_status_pub_->publish(status_msg);

    geometry_msgs::msg::Twist twist;
    if (enable_motion_) {
      twist.linear.x = mission_out.cmd_linear_x_m_s;
      twist.angular.z = mission_out.cmd_angular_z_rad_s;
    }
    cmd_vel_pub_->publish(twist);

    if (enable_motion_ && !follow_only_) {
      relbot_interfaces::msg::ArmCommand arm_msg;
      arm_msg.header.stamp = now;
      arm_msg.arm_angle_rad = mission_out.arm_angle_rad;
      arm_msg.arm_velocity_rad_s = mission_out.arm_velocity_rad_s;
      arm_msg.gripper_angle_deg = mission_out.gripper_angle_deg;
      arm_msg.torque_enable = mission_out.torque_enable;
      arm_cmd_pub_->publish(arm_msg);
    }

    // Periodic concise status log
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[frame %lu] %s | ball=%s (d=%.2fm, b=%.1fdeg, c=%.2f) | cmd=(v=%.2f, w=%.2f) [%s]",
        frame_count_, state_name,
        pickup_det.valid ? "YES" : "no", pickup_det.estimated_distance_m,
        pickup_det.estimated_bearing_rad * kRadToDeg, pickup_det.confidence,
        mission_out.cmd_linear_x_m_s, mission_out.cmd_angular_z_rad_s,
        enable_motion_ ? "LIVE" : "SAFE");

    // Annotate debug frame
    cv::line(frame_bgr, cv::Point(frame_bgr.cols / 2, 0),
             cv::Point(frame_bgr.cols / 2, frame_bgr.rows - 1),
             cv::Scalar(80, 80, 80), 1);
    drawDetectionBox(frame_bgr, pickup_det, cv::Scalar(0, 255, 0), "BALL");
    if (!follow_only_) {
      drawDetectionBox(frame_bgr, delivery_det, cv::Scalar(255, 140, 0), "DELIVERY");
    }

    char hud[160];
    std::snprintf(hud, sizeof(hud), "%s | v=%.2f w=%.2f [%s]",
                  state_name, mission_out.cmd_linear_x_m_s, mission_out.cmd_angular_z_rad_s,
                  enable_motion_ ? "LIVE" : "SAFE");
    cv::rectangle(frame_bgr, cv::Point(0, frame_bgr.rows - 20),
                  cv::Point(frame_bgr.cols, frame_bgr.rows), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(frame_bgr, hud, cv::Point(4, frame_bgr.rows - 6),
                cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

    // Publish ROS debug image
    sensor_msgs::msg::Image dbg_msg;
    dbg_msg.header.stamp = now;
    dbg_msg.header.frame_id = "camera_link";
    dbg_msg.height = static_cast<uint32_t>(frame_bgr.rows);
    dbg_msg.width = static_cast<uint32_t>(frame_bgr.cols);
    dbg_msg.encoding = "bgr8";
    dbg_msg.is_bigendian = 0;
    dbg_msg.step = static_cast<uint32_t>(frame_bgr.cols * 3);
    dbg_msg.data.assign(frame_bgr.data, frame_bgr.data + frame_bgr.total() * frame_bgr.elemSize());
    debug_image_pub_->publish(dbg_msg);

    // Encode JPEG for HTTP MJPEG stream & periodic snapshot
    if (preview_server_ || (!snapshot_path_.empty() && frame_count_ % 10 == 1)) {
      std::vector<uint8_t> jpg;
      std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 75};
      if (cv::imencode(".jpg", frame_bgr, jpg, params)) {
        if (preview_server_) {
          preview_server_->updateFrame(jpg);
        }
        if (!snapshot_path_.empty() && frame_count_ % 10 == 1) {
          const std::string tmp_path = snapshot_path_ + ".tmp";
          std::ofstream ofs(tmp_path, std::ios::binary);
          if (ofs) {
            ofs.write(reinterpret_cast<const char*>(jpg.data()),
                      static_cast<std::streamsize>(jpg.size()));
            ofs.close();
            std::rename(tmp_path.c_str(), snapshot_path_.c_str());
          }
        }
      }
    }
  }

  std::string camera_source_;
  int camera_device_{0};
  std::string image_topic_;
  int capture_width_{1280};
  int capture_height_{960};
  bool use_mjpg_{true};
  int target_width_{320};
  int target_height_{240};
  bool rotate_180_{true};
  double fps_{20.0};
  bool enable_motion_{false};
  bool follow_only_{false};
  bool manual_mode_{false};
  std::mutex manual_mutex_;
  double manual_linear_x_{0.0};
  double manual_angular_z_{0.0};
  rclcpp::Time last_manual_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_manual_cmd_{false};
  bool search_when_lost_{false};
  double reacquire_timeout_sec_{1.5};
  double min_ball_confidence_{0.42};
  double follow_standoff_m_{0.25};
  int http_port_{8080};
  std::string snapshot_path_;
  uint64_t frame_count_{0};
  int consecutive_ball_frames_{0};
  bool has_seen_ball_{false};
  double last_seen_bearing_rad_{0.0};
  rclcpp::Time last_seen_time_{0, 0, RCL_ROS_TIME};

  cv::VideoCapture cap_;
  std::unique_ptr<MjpegPreviewServer> preview_server_;

  RelbotNavigationConfigC nav_cfg_{};
  RelbotDetectorHandle* detector_{nullptr};
  RelbotMissionHandle* mission_{nullptr};
  RelbotPose2DC latest_odom_{};
  RelbotManipulatorStateC latest_manip_{};
  rclcpp::Time last_step_time_;

  rclcpp::Publisher<relbot_interfaces::msg::DetectedObject>::SharedPtr detected_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
  rclcpp::Publisher<relbot_interfaces::msg::MissionStatus>::SharedPtr mission_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<relbot_interfaces::msg::ArmCommand>::SharedPtr arm_cmd_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<relbot_interfaces::msg::ManipulatorState>::SharedPtr manip_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_sub_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
};

}  // namespace relbot::nav

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<relbot::nav::RelbotPerceptionNavNode>());
  rclcpp::shutdown();
  return 0;
}
