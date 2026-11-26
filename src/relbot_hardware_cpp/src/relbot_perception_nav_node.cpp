#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
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

// Standard OpenCV DICT_4X4_50 first 20 16-bit codes (row-major 4x4 inner bits, 1=white, 0=black).
// Note: ID 2 is 0x2A55 (0010 1010 0101 0101), matching the 40x40mm ArUco cube #2!
constexpr std::array<uint16_t, 20> kDict4x4Codes = {
    0xD562, 0x990F, 0x2A55, 0x663A, 0xB8CC, 0x6A33, 0x16B9, 0xC6EE, 0x8F87, 0x05F9,
    0xB5E1, 0xF78E, 0xE2F4, 0x628D, 0xDB39, 0x2C7F, 0x57B2, 0x9861, 0x8B7C, 0x17F4};

uint16_t rotate4x4Bits90CW(uint16_t code) {
  uint16_t out = 0;
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      const int src_r = 3 - c;
      const int src_c = r;
      const int src_bit = (code >> (15 - (src_r * 4 + src_c))) & 1;
      out = static_cast<uint16_t>((out << 1) | src_bit);
    }
  }
  return out;
}

int popcount16(uint16_t v) {
  int c = 0;
  while (v) {
    v &= static_cast<uint16_t>(v - 1);
    ++c;
  }
  return c;
}

struct ArucoCubeDetection {
  bool valid{false};
  int marker_id{-1};
  std::array<cv::Point2f, 4> corners_norm{};  // Normalized [0..1] quad corners
  RelbotDetectedObjectC det{};
};

// Fast, dependency-free 4x4 ArUco square fiducial detector operating on the 640x480 frame
ArucoCubeDetection detectArucoCube4x4(const cv::Mat& bgr_full,
                                      double marker_size_m,
                                      double hfov_rad,
                                      double vfov_rad) {
  ArucoCubeDetection result;
  if (bgr_full.empty()) {
    return result;
  }

  cv::Mat gray;
  cv::cvtColor(bgr_full, gray, cv::COLOR_BGR2GRAY);

  cv::Mat thresh;
  cv::adaptiveThreshold(gray, thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                        cv::THRESH_BINARY_INV, 21, 7);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  const double img_w = static_cast<double>(bgr_full.cols);
  const double img_h = static_cast<double>(bgr_full.rows);
  const double focal_y_px = (img_h * 0.5) / std::tan(vfov_rad * 0.5);

  double best_score = -1.0;

  for (const auto& contour : contours) {
    const double perim = cv::arcLength(contour, true);
    if (perim < 44.0 || perim > (img_w * 1.8)) {
      continue;
    }

    std::vector<cv::Point> poly;
    cv::approxPolyDP(contour, poly, 0.045 * perim, true);
    if (poly.size() != 4 || !cv::isContourConvex(poly)) {
      continue;
    }

    const double area = std::abs(cv::contourArea(poly));
    if (area < 120.0 || area > (img_w * img_h * 0.45)) {
      continue;
    }

    const cv::Rect bbox = cv::boundingRect(poly);
    const double aspect = static_cast<double>(bbox.width) / std::max(1, bbox.height);
    if (aspect < 0.50 || aspect > 2.00) {
      continue;
    }

    // Order corners clockwise: top-left, top-right, bottom-right, bottom-left
    std::array<cv::Point2f, 4> pts;
    for (int i = 0; i < 4; ++i) {
      pts[i] = cv::Point2f(static_cast<float>(poly[i].x), static_cast<float>(poly[i].y));
    }
    // Ensure clockwise winding
    const float cross = (pts[1].x - pts[0].x) * (pts[2].y - pts[0].y) -
                        (pts[1].y - pts[0].y) * (pts[2].x - pts[0].x);
    if (cross < 0.0f) {
      std::swap(pts[1], pts[3]);
    }
    // Rotate array so pts[0] has minimum (x + y)
    int tl_idx = 0;
    float min_sum = pts[0].x + pts[0].y;
    for (int i = 1; i < 4; ++i) {
      if (pts[i].x + pts[i].y < min_sum) {
        min_sum = pts[i].x + pts[i].y;
        tl_idx = i;
      }
    }
    std::array<cv::Point2f, 4> ordered;
    for (int i = 0; i < 4; ++i) {
      ordered[i] = pts[(tl_idx + i) % 4];
    }

    // Warp quad to 48x48 canonical patch (6x6 cells of 8x8 pixels; outer ring = black border, inner 4x4 = data)
    const std::array<cv::Point2f, 4> dst_pts = {
        cv::Point2f(0.0f, 0.0f), cv::Point2f(47.0f, 0.0f),
        cv::Point2f(47.0f, 47.0f), cv::Point2f(0.0f, 47.0f)};
    cv::Mat H = cv::getPerspectiveTransform(ordered.data(), dst_pts.data());
    cv::Mat warped;
    cv::warpPerspective(gray, warped, H, cv::Size(48, 48), cv::INTER_LINEAR);

    double min_v = 0.0, max_v = 0.0;
    cv::minMaxLoc(warped, &min_v, &max_v);
    if (max_v - min_v < 45.0) {
      continue;  // Low contrast patch
    }
    const double mid_thresh = 0.5 * (min_v + max_v);

    int border_black_cells = 0;
    double border_sum = 0.0;
    int white_data_bits = 0;
    uint16_t code16 = 0;

    for (int r = 0; r < 6; ++r) {
      for (int c = 0; c < 6; ++c) {
        // Sample center 4x4 pixels of the 8x8 cell
        cv::Rect cell_roi(c * 8 + 2, r * 8 + 2, 4, 4);
        const double cell_mean = cv::mean(warped(cell_roi))[0];
        const bool is_white = (cell_mean > mid_thresh);

        if (r == 0 || r == 5 || c == 0 || c == 5) {
          border_sum += cell_mean;
          if (!is_white) {
            ++border_black_cells;
          }
        } else {
          code16 = static_cast<uint16_t>((code16 << 1) | (is_white ? 1 : 0));
          if (is_white) {
            ++white_data_bits;
          }
        }
      }
    }

    // Must have a solid black 1-cell border (at least 18 of 20 cells dark) and non-trivial inner 4x4 data
    if (border_black_cells < 18 || white_data_bits < 3 || white_data_bits > 13) {
      continue;
    }

    // Check that the marker sits on a brighter background (the white 40x40mm cube face)
    const double border_mean = border_sum / 20.0;
    cv::Rect expanded_box(
        std::max(0, bbox.x - std::max(2, bbox.width / 6)),
        std::max(0, bbox.y - std::max(2, bbox.height / 6)),
        std::min(bgr_full.cols - std::max(0, bbox.x - std::max(2, bbox.width / 6)),
                 bbox.width + 2 * std::max(2, bbox.width / 6)),
        std::min(bgr_full.rows - std::max(0, bbox.y - std::max(2, bbox.height / 6)),
                 bbox.height + 2 * std::max(2, bbox.height / 6)));
    const double surround_mean = cv::mean(gray(expanded_box))[0];
    if (surround_mean < border_mean + 12.0) {
      continue;
    }

    // Match inner 16-bit code against DICT_4X4_50 across all 4 rotations
    int matched_id = -1;
    int best_hamming = 16;
    uint16_t rot_code = code16;
    for (int rot = 0; rot < 4; ++rot) {
      for (size_t id = 0; id < kDict4x4Codes.size(); ++id) {
        const int ham = popcount16(static_cast<uint16_t>(rot_code ^ kDict4x4Codes[id]));
        if (ham < best_hamming) {
          best_hamming = ham;
          matched_id = static_cast<int>(id);
        }
      }
      rot_code = rotate4x4Bits90CW(rot_code);
    }

    if (best_hamming > 1) {
      continue;  // Only accept exact DICT_4X4 matches (or 1-bit motion-blur tolerance)
    }

    const double dict_bonus = (best_hamming == 0) ? 2.5 : 1.5;
    const double score = area * dict_bonus * (static_cast<double>(border_black_cells) / 20.0);
    if (score > best_score) {
      best_score = score;
      result.valid = true;
      result.marker_id = matched_id;

      float cx_px = 0.0f, cy_px = 0.0f;
      for (int i = 0; i < 4; ++i) {
        cx_px += ordered[i].x * 0.25f;
        cy_px += ordered[i].y * 0.25f;
        result.corners_norm[i] = cv::Point2f(
            ordered[i].x / static_cast<float>(img_w),
            ordered[i].y / static_cast<float>(img_h));
      }

      // Apparent marker pixel span (combining bounding box and area root for rotation stability)
      const double side_px = 0.5 * (std::max(bbox.width, bbox.height) + std::sqrt(area));
      const double dist_m = std::clamp((marker_size_m * focal_y_px) / std::max(4.0, side_px), 0.06, 2.50);
      const float norm_cx = static_cast<float>((cx_px / img_w) * 2.0 - 1.0);
      const float norm_cy = static_cast<float>((cy_px / img_h) * 2.0 - 1.0);
      const double bearing_rad = -static_cast<double>(norm_cx) * (hfov_rad * 0.5);

      result.det.valid = true;
      result.det.target_class = 0;
      result.det.confidence = (best_hamming == 0) ? 0.98f : 0.85f;
      result.det.bbox_center_x = norm_cx;
      result.det.bbox_center_y = norm_cy;
      result.det.bbox_width = static_cast<float>(bbox.width / img_w);
      result.det.bbox_height = static_cast<float>(bbox.height / img_h);
      result.det.estimated_distance_m = dist_m;
      result.det.estimated_bearing_rad = bearing_rad;
    }
  }

  return result;
}

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
          "<title>RELbot ArUco Cube Pick-Up & Servo Teleop</title>"
          "<style>body{background:#111;color:#eee;font-family:monospace;text-align:center;margin:12px;}"
          "img{border:2px solid #00e5ff;max-width:95%;width:520px;height:auto;border-radius:6px;}"
          ".btns{margin:8px auto;display:inline-grid;grid-template-columns:repeat(3,105px);gap:6px;}"
          ".servo-btns{margin:8px auto;display:inline-grid;grid-template-columns:repeat(4,120px);gap:6px;}"
          "button{background:#222;color:#eee;border:1px solid #555;padding:10px;font-family:monospace;"
          "font-size:13px;border-radius:6px;cursor:pointer;user-select:none;}"
          "button:active,button.active{background:#00e5ff;color:#000;font-weight:bold;}"
          ".mode-bar{margin:8px;}"
          "</style></head><body>"
          "<h2>RELbot ArUco Cube Pick-Up & ESP32 Servo Control</h2>"
          "<img src=\"/stream\" alt=\"RELbot live stream\"/>"
          "<div class=\"mode-bar\">"
          "<button onclick=\"setMode('auto')\" style=\"border-color:#00e5ff;\">AUTO (Find & Pick ArUco Cube)</button> "
          "<button onclick=\"setMode('manual')\">MANUAL (WASD + Servo Keys)</button> "
          "<button onclick=\"api('macro=pickup')\" style=\"background:#1b5e20;\">PICKUP MACRO (G)</button> "
          "<button onclick=\"api('macro=release')\" style=\"background:#4a148c;\">RELEASE CUBE (R)</button>"
          "<div id=\"status\" style=\"margin-top:6px;color:#00e5ff;\">Keys: WASD=Drive | I/K=Arm Up/Down | O/P=Grip Open(g 0)/Close(g 180) | G=Pickup | R=Release</div>"
          "</div>"
          "<div class=\"btns\">"
          "<div></div><button onmousedown=\"drv(0.16,0)\" onmouseup=\"drv(0,0)\" ontouchstart=\"drv(0.16,0)\" ontouchend=\"drv(0,0)\">W / ▲</button><div></div>"
          "<button onmousedown=\"drv(0,0.55)\" onmouseup=\"drv(0,0)\" ontouchstart=\"drv(0,0.55)\" ontouchend=\"drv(0,0)\">A / ◀</button>"
          "<button onclick=\"drv(0,0)\" style=\"background:#b71c1c;\">STOP</button>"
          "<button onmousedown=\"drv(0,-0.55)\" onmouseup=\"drv(0,0)\" ontouchstart=\"drv(0,-0.55)\" ontouchend=\"drv(0,0)\">D / ▶</button>"
          "<div></div><button onmousedown=\"drv(-0.14,0)\" onmouseup=\"drv(0,0)\" ontouchstart=\"drv(-0.14,0)\" ontouchend=\"drv(0,0)\">S / ▼</button><div></div>"
          "</div><br/>"
          "<div class=\"servo-btns\">"
          "<button onclick=\"api('servo=arm_up')\">Arm +10° (I)</button>"
          "<button onclick=\"api('servo=arm_down')\">Arm -10° (K)</button>"
          "<button onclick=\"api('servo=arm_pick')\">Arm Lower -40° (U)</button>"
          "<button onclick=\"api('servo=arm_carry')\">Arm Carry +20° (Y)</button>"
          "<button onclick=\"api('servo=grip_open')\" style=\"border-color:#8bc34a;\">Grip Open g 0 (O)</button>"
          "<button onclick=\"api('servo=grip_close')\" style=\"border-color:#ffb300;\">Grip Close g 180 (P)</button>"
          "<button onclick=\"api('servo=arm_zero')\">Arm 0° (0)</button>"
          "<button onclick=\"api('servo=grip_stop')\" style=\"background:#b71c1c;\">Grip OFF x (X)</button>"
          "</div>"
          "<script>"
          "let tmr=null;"
          "function api(q){fetch('/api/cmd?'+q).then(r=>r.json()).then(d=>{"
          "document.getElementById('status').innerText='Mode: '+(d.manual?'MANUAL':'AUTO')+' | v='+d.v.toFixed(2)+' w='+d.w.toFixed(2)+' | Arm='+d.arm.toFixed(0)+'deg Grip='+d.grip;"
          "}).catch(()=>{});}"
          "function setMode(m){api('mode='+m);}"
          "function drv(v,w){if(tmr){clearInterval(tmr);tmr=null;}api('mode=manual&v='+v+'&w='+w);"
          "if(v!==0||w!==0){tmr=setInterval(()=>api('mode=manual&v='+v+'&w='+w),250);}}"
          "const keys={};"
          "function updKey(){"
          "let v=0,w=0;"
          "if(keys['KeyW']||keys['ArrowUp'])v+=0.16;"
          "if(keys['KeyS']||keys['ArrowDown'])v-=0.14;"
          "if(keys['KeyA']||keys['ArrowLeft'])w+=0.55;"
          "if(keys['KeyD']||keys['ArrowRight'])w-=0.55;"
          "drv(v,w);}"
          "window.addEventListener('keydown',e=>{if(e.repeat)return;"
          "if(e.code==='Space'){Object.keys(keys).forEach(k=>delete keys[k]);drv(0,0);e.preventDefault();return;}"
          "if(e.code==='KeyM'){setMode('toggle');return;}"
          "if(e.code==='KeyI'){api('servo=arm_up');return;}"
          "if(e.code==='KeyK'){api('servo=arm_down');return;}"
          "if(e.code==='KeyU'){api('servo=arm_pick');return;}"
          "if(e.code==='KeyY'){api('servo=arm_carry');return;}"
          "if(e.code==='Digit0'){api('servo=arm_zero');return;}"
          "if(e.code==='KeyO'){api('servo=grip_open');return;}"
          "if(e.code==='KeyP'){api('servo=grip_close');return;}"
          "if(e.code==='KeyX'){api('servo=grip_stop');return;}"
          "if(e.code==='KeyG'){api('macro=pickup');return;}"
          "if(e.code==='KeyR'){api('macro=release');return;}"
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
  enum class AutoPickupStage {
    kSearchOrApproach = 0,
    kOpenAndCreep,
    kLowerArm,
    kCloseGripper,
    kLiftArm,
    kHoldingCube,
    kReleaseLower,
    kReleaseOpen,
    kReleaseLift
  };

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
    search_when_lost_ = this->declare_parameter<bool>("search_when_lost", true);
    reacquire_timeout_sec_ = this->declare_parameter<double>("reacquire_timeout_sec", 0.65);
    min_ball_confidence_ = this->declare_parameter<double>("min_ball_confidence", 0.42);
    follow_standoff_m_ = this->declare_parameter<double>("follow_standoff_m", 0.19);
    http_port_ = this->declare_parameter<int>("http_port", 8080);
    snapshot_path_ = this->declare_parameter<std::string>("snapshot_path", "/tmp/relbot_debug.jpg");

    // ESP32 Servo & ArUco Cube Pick-Up Parameters
    esp32_serial_port_ = this->declare_parameter<std::string>("esp32_serial_port", "/dev/ttyUSB0");
    auto_pickup_cube_ = this->declare_parameter<bool>("auto_pickup_cube", true);
    aruco_marker_size_m_ = this->declare_parameter<double>("aruco_marker_size_m", 0.030);
    arm_carry_deg_ = this->declare_parameter<double>("arm_carry_deg", 20.0);
    arm_pickup_deg_ = this->declare_parameter<double>("arm_pickup_deg", -40.0);
    grip_open_cmd_ = this->declare_parameter<int>("grip_open_cmd", 0);
    grip_close_cmd_ = this->declare_parameter<int>("grip_close_cmd", 180);

    // Configure Rust perception parameters (fallback green ball + blue delivery zone)
    RelbotPerceptionConfigC pcfg = relbot_perception_config_default();
    pcfg.image_width = static_cast<uint32_t>(target_width_);
    pcfg.image_height = static_cast<uint32_t>(target_height_);
    pcfg.horizontal_fov_rad = this->declare_parameter<double>("horizontal_fov_rad", pcfg.horizontal_fov_rad);
    pcfg.vertical_fov_rad = this->declare_parameter<double>("vertical_fov_rad", pcfg.vertical_fov_rad);
    pcfg.pickup_object_height_m = this->declare_parameter<double>("pickup_object_height_m", 0.040);
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

    hfov_rad_ = pcfg.horizontal_fov_rad;
    vfov_rad_ = pcfg.vertical_fov_rad;
    detector_ = relbot_detector_create(&pcfg);

    // Configure Rust navigation parameters with stable damped angular gains
    nav_cfg_ = relbot_navigation_config_default();
    nav_cfg_.pickup_standoff_m = this->declare_parameter<double>("pickup_standoff_m", 0.19);
    nav_cfg_.delivery_standoff_m = this->declare_parameter<double>("delivery_standoff_m", nav_cfg_.delivery_standoff_m);
    nav_cfg_.kp_linear = this->declare_parameter<double>("kp_linear", 0.45);
    nav_cfg_.kp_angular = this->declare_parameter<double>("kp_angular", 1.15);
    nav_cfg_.max_linear_speed_m_s = this->declare_parameter<double>("max_linear_speed_m_s", 0.15);
    nav_cfg_.max_angular_speed_rad_s = this->declare_parameter<double>("max_angular_speed_rad_s", 0.55);
    nav_cfg_.search_spin_rad_s = this->declare_parameter<double>("search_spin_rad_s", 0.28);

    mission_ = relbot_mission_create(&nav_cfg_);

    // Open ESP32 Serial Bridge (/dev/ttyUSB0) without toggling DTR/RTS (~HUPCL)
    initEsp32Serial();

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
                    "Live ArUco MJPEG preview + Servo UI at http://<relbot-ip>:%d/",
                    http_port_);
      } else {
        RCLCPP_WARN(this->get_logger(), "Could not start HTTP preview on port %d", http_port_);
      }
    }

    last_step_time_ = this->now();
    stage_start_time_ = this->now();

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
                    "Opened /dev/video%d capture=%dx%d -> ArUco=640x480, HSV=%dx%d (rotate_180=%s, enable_motion=%s)",
                    camera_device_, actual_w, actual_h,
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
    }
  }

  ~RelbotPerceptionNavNode() override {
    if (preview_server_) {
      preview_server_->stop();
    }
    if (cap_.isOpened()) {
      cap_.release();
    }
    if (esp32_fd_ >= 0) {
      ::close(esp32_fd_);
      esp32_fd_ = -1;
    }
    relbot_detector_destroy(detector_);
    relbot_mission_destroy(mission_);
  }

 private:
  void initEsp32Serial() {
    std::lock_guard<std::mutex> lock(esp32_mutex_);
    if (esp32_serial_port_.empty() || esp32_serial_port_ == "/dev/null") {
      return;
    }
    esp32_fd_ = ::open(esp32_serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (esp32_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "Could not open ESP32 serial port %s", esp32_serial_port_.c_str());
      return;
    }
    struct termios tty {};
    if (tcgetattr(esp32_fd_, &tty) == 0) {
      cfsetospeed(&tty, B115200);
      cfsetispeed(&tty, B115200);
      tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
      tty.c_cflag |= (CLOCAL | CREAD);
      tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS | HUPCL);
      tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY);
      tty.c_lflag = 0;
      tty.c_oflag = 0;
      tcsetattr(esp32_fd_, TCSANOW, &tty);
    }
    RCLCPP_INFO(this->get_logger(), "Connected ESP32 servo toolkit on %s @ 115200 baud",
                esp32_serial_port_.c_str());
    current_arm_deg_ = arm_carry_deg_;
    sendEsp32RawUnlocked("a " + std::to_string(static_cast<int>( std::round(current_arm_deg_))));
  }

  void sendEsp32RawUnlocked(const std::string& cmd) {
    if (esp32_fd_ < 0) {
      esp32_fd_ = ::open(esp32_serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    }
    if (esp32_fd_ >= 0) {
      const std::string line = cmd + "\n";
      const ssize_t n = ::write(esp32_fd_, line.data(), line.size());
      (void)n;
      // Drain any unread serial echo lines so kernel buffer never fills
      char drain[256];
      while (::read(esp32_fd_, drain, sizeof(drain)) > 0) {
      }
    }
  }

  void sendEsp32Command(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(esp32_mutex_);
    sendEsp32RawUnlocked(cmd);
    RCLCPP_INFO(this->get_logger(), "[ESP32 Servo] Sent: '%s'", cmd.c_str());
  }

  void setArmDeg(double deg) {
    current_arm_deg_ = std::clamp(deg, -65.0, 65.0);
    char buf[32];
    std::snprintf(buf, sizeof(buf), "a %.1f", current_arm_deg_);
    sendEsp32Command(buf);
  }

  void triggerGripDeg(int deg) {
    last_grip_label_ = (deg >= 135) ? "CLOSE(180)" : ((deg <= 45) ? "OPEN(0)" : "MID(90)");
    sendEsp32Command("g " + std::to_string(deg));
  }

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
      return;
    }
    processBgrFrame(frame_bgr);
  }

  void drawDetectionBox(cv::Mat& img, const RelbotDetectedObjectC& det,
                        const cv::Scalar& color, const std::string& label,
                        const ArucoCubeDetection* aruco = nullptr) {
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

    if (aruco && aruco->valid) {
      for (int i = 0; i < 4; ++i) {
        const cv::Point p1(static_cast<int>(aruco->corners_norm[i].x * w),
                           static_cast<int>(aruco->corners_norm[i].y * h));
        const cv::Point p2(static_cast<int>(aruco->corners_norm[(i + 1) % 4].x * w),
                           static_cast<int>(aruco->corners_norm[(i + 1) % 4].y * h));
        cv::line(img, p1, p2, color, 2, cv::LINE_AA);
      }
    } else {
      cv::rectangle(img, cv::Point(x0, y0), cv::Point(x1, y1), color, 2);
    }
    cv::drawMarker(img, cv::Point(cx, cy), color, cv::MARKER_CROSS, 12, 2);

    char info[128];
    std::snprintf(info, sizeof(info), "%s d=%.2fm b=%.1fdeg",
                  label.c_str(), det.estimated_distance_m,
                  det.estimated_bearing_rad * kRadToDeg);
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
      if (pickup_stage_ == AutoPickupStage::kHoldingCube) {
        pickup_stage_ = AutoPickupStage::kSearchOrApproach;
        setArmDeg(arm_carry_deg_);
      }
    } else if (mode == "toggle") {
      manual_mode_ = !manual_mode_;
      manual_linear_x_ = 0.0;
      manual_angular_z_ = 0.0;
    }

    const std::string servo = getParam("servo");
    if (servo == "arm_up") {
      setArmDeg(current_arm_deg_ + 10.0);
    } else if (servo == "arm_down") {
      setArmDeg(current_arm_deg_ - 10.0);
    } else if (servo == "arm_pick") {
      setArmDeg(arm_pickup_deg_);
    } else if (servo == "arm_carry") {
      setArmDeg(arm_carry_deg_);
    } else if (servo == "arm_zero") {
      setArmDeg(0.0);
    } else if (servo == "grip_open") {
      triggerGripDeg(grip_open_cmd_);
    } else if (servo == "grip_close") {
      triggerGripDeg(grip_close_cmd_);
    } else if (servo == "grip_stop") {
      last_grip_label_ = "OFF";
      sendEsp32Command("x");
    }

    const std::string raw = getParam("raw");
    if (!raw.empty()) {
      sendEsp32Command(raw);
    }

    const std::string macro = getParam("macro");
    if (macro == "pickup") {
      pickup_stage_ = AutoPickupStage::kLowerArm;
      stage_start_time_ = this->now();
      setArmDeg(arm_pickup_deg_);
    } else if (macro == "release") {
      pickup_stage_ = AutoPickupStage::kReleaseLower;
      stage_start_time_ = this->now();
      setArmDeg(arm_pickup_deg_ * 0.75);
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

    char buf[196];
    std::snprintf(buf, sizeof(buf),
                  "{\"ok\":true,\"manual\":%s,\"v\":%.3f,\"w\":%.3f,\"arm\":%.1f,\"grip\":\"%s\"}",
                  manual_mode_ ? "true" : "false", manual_linear_x_, manual_angular_z_,
                  current_arm_deg_, last_grip_label_.c_str());
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

    // 1. Run high-resolution 4x4 ArUco Cube detection on the full 640x480 frame first!
    const ArucoCubeDetection aruco_det = detectArucoCube4x4(
        frame_bgr, aruco_marker_size_m_, hfov_rad_, vfov_rad_);

    if (frame_bgr.cols != target_width_ || frame_bgr.rows != target_height_) {
      cv::resize(frame_bgr, frame_bgr, cv::Size(target_width_, target_height_));
    }

    cv::Mat frame_rgb;
    cv::cvtColor(frame_bgr, frame_rgb, cv::COLOR_BGR2RGB);
    if (!frame_rgb.isContinuous()) {
      frame_rgb = frame_rgb.clone();
    }

    const size_t rgb_bytes = frame_rgb.total() * frame_rgb.elemSize();
    RelbotDetectedObjectC pickup_det{};
    std::string target_label = "TARGET";

    if (aruco_det.valid) {
      pickup_det = aruco_det.det;
      target_label = (aruco_det.marker_id >= 0)
                         ? ("ARUCO #" + std::to_string(aruco_det.marker_id))
                         : "ARUCO_CUBE";
    } else {
      // Fallback to green ball/object detection if no ArUco marker is visible
      pickup_det = relbot_detector_detect_rgb8(
          detector_, frame_rgb.data, rgb_bytes,
          static_cast<uint32_t>(frame_rgb.cols), static_cast<uint32_t>(frame_rgb.rows), 0);
      if (pickup_det.valid &&
          (pickup_det.confidence < static_cast<float>(min_ball_confidence_) ||
           pickup_det.estimated_distance_m > 2.5)) {
        pickup_det.valid = false;
      }
      if (pickup_det.valid) {
        target_label = "BALL";
      }
    }

    const rclcpp::Time now = this->now();
    double dt = (now - last_step_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) {
      dt = 0.05;
    }
    last_step_time_ = now;

    ++frame_count_;
    const bool warmed_up = frame_count_ > 15;

    // Smooth target tracking & 0.65s persistence memory so brief ArUco motion blur never causes spin-away
    if (warmed_up && pickup_det.valid) {
      ++consecutive_ball_frames_;
      if (!has_seen_ball_ || (now - last_seen_time_).seconds() > 1.2) {
        smoothed_distance_m_ = pickup_det.estimated_distance_m;
        smoothed_bearing_rad_ = pickup_det.estimated_bearing_rad;
      } else {
        smoothed_distance_m_ = 0.55 * smoothed_distance_m_ + 0.45 * pickup_det.estimated_distance_m;
        smoothed_bearing_rad_ = 0.55 * smoothed_bearing_rad_ + 0.45 * pickup_det.estimated_bearing_rad;
      }
      pickup_det.estimated_distance_m = smoothed_distance_m_;
      pickup_det.estimated_bearing_rad = smoothed_bearing_rad_;
      last_valid_det_ = pickup_det;
      last_valid_label_ = target_label;
      has_seen_ball_ = true;
      last_seen_bearing_rad_ = smoothed_bearing_rad_;
      last_seen_time_ = now;
    } else {
      consecutive_ball_frames_ = 0;
    }

    const double since_seen = has_seen_ball_ ? (now - last_seen_time_).seconds() : 999.0;
    const bool target_active = warmed_up && (pickup_det.valid || (since_seen <= reacquire_timeout_sec_));
    const RelbotDetectedObjectC active_det = pickup_det.valid ? pickup_det : last_valid_det_;

    RelbotMissionOutputC mission_out{};
    const char* state_name = "IDLE";

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

    // Execute active pick-up or release macro stages (works in both AUTO and MANUAL when triggered)
    const double stage_elapsed = (now - stage_start_time_).seconds();
    if (pickup_stage_ == AutoPickupStage::kOpenAndCreep) {
      mission_out.cmd_linear_x_m_s = (stage_elapsed < 0.40) ? 0.07 : 0.0;
      mission_out.cmd_angular_z_rad_s = 0.0;
      state_name = "PICK_1_OPEN_CREEP";
      if (stage_elapsed >= 0.55) {
        pickup_stage_ = AutoPickupStage::kLowerArm;
        stage_start_time_ = now;
        setArmDeg(arm_pickup_deg_);
      }
    } else if (pickup_stage_ == AutoPickupStage::kLowerArm) {
      mission_out.cmd_linear_x_m_s = 0.0;
      mission_out.cmd_angular_z_rad_s = 0.0;
      state_name = "PICK_2_LOWER_ARM";
      if (stage_elapsed >= 0.75) {
        pickup_stage_ = AutoPickupStage::kCloseGripper;
        stage_start_time_ = now;
        triggerGripDeg(grip_close_cmd_);
      }
    } else if (pickup_stage_ == AutoPickupStage::kCloseGripper) {
      mission_out.cmd_linear_x_m_s = 0.0;
      mission_out.cmd_angular_z_rad_s = 0.0;
      state_name = "PICK_3_GRIP_180";
      if (stage_elapsed >= 0.65) {
        pickup_stage_ = AutoPickupStage::kLiftArm;
        stage_start_time_ = now;
        setArmDeg(arm_carry_deg_ + 10.0);
      }
    } else if (pickup_stage_ == AutoPickupStage::kLiftArm) {
      mission_out.cmd_linear_x_m_s = 0.0;
      mission_out.cmd_angular_z_rad_s = 0.0;
      state_name = "PICK_4_LIFT_CUBE";
      if (stage_elapsed >= 0.75) {
        pickup_stage_ = AutoPickupStage::kHoldingCube;
        stage_start_time_ = now;
      }
    } else if (pickup_stage_ == AutoPickupStage::kReleaseLower) {
      mission_out.cmd_linear_x_m_s = 0.0;
      mission_out.cmd_angular_z_rad_s = 0.0;
      state_name = "REL_1_LOWER_ARM";
      if (stage_elapsed >= 0.65) {
        pickup_stage_ = AutoPickupStage::kReleaseOpen;
        stage_start_time_ = now;
        triggerGripDeg(grip_open_cmd_);
      }
    } else if (pickup_stage_ == AutoPickupStage::kReleaseOpen) {
      mission_out.cmd_linear_x_m_s = 0.0;
      mission_out.cmd_angular_z_rad_s = 0.0;
      state_name = "REL_2_OPEN_GRIP_0";
      if (stage_elapsed >= 0.60) {
        pickup_stage_ = AutoPickupStage::kReleaseLift;
        stage_start_time_ = now;
        setArmDeg(arm_carry_deg_);
      }
    } else if (pickup_stage_ == AutoPickupStage::kReleaseLift) {
      mission_out.cmd_linear_x_m_s = 0.0;
      mission_out.cmd_angular_z_rad_s = 0.0;
      state_name = "REL_3_STOW_ARM";
      if (stage_elapsed >= 0.60) {
        pickup_stage_ = AutoPickupStage::kSearchOrApproach;
        stage_start_time_ = now;
      }
    } else if (is_manual) {
      mission_out.status_code = 0;
      mission_out.cmd_linear_x_m_s = man_v;
      mission_out.cmd_angular_z_rad_s = man_w;
      mission_out.distance_to_goal_m = pickup_det.valid ? pickup_det.estimated_distance_m : 0.0;
      state_name = "MANUAL_CTRL";
    } else if (pickup_stage_ == AutoPickupStage::kHoldingCube) {
      mission_out.status_code = 3;
      mission_out.cmd_linear_x_m_s = 0.0;
      mission_out.cmd_angular_z_rad_s = 0.0;
      mission_out.holding_object = true;
      state_name = "HOLDING_CUBE (press R to release)";
    } else {
      // Autonomous ArUco Cube Search -> Damped Approach -> Pick-Up
      if (!warmed_up) {
        state_name = "CAM_WARMUP";
      } else if (target_active) {
        const double dist = active_det.estimated_distance_m;
        const double bear = active_det.estimated_bearing_rad;
        mission_out.distance_to_goal_m = dist;

        // Check if within pickup standoff & bearing tolerance
        if (auto_pickup_cube_ && dist <= (follow_standoff_m_ + 0.025) && std::abs(bear) <= 0.10) {
          ++aligned_pickup_frames_;
          mission_out.cmd_linear_x_m_s = 0.0;
          mission_out.cmd_angular_z_rad_s = 0.0;
          state_name = "ALIGNED_FOR_CUBE";
          if (aligned_pickup_frames_ >= 3) {
            aligned_pickup_frames_ = 0;
            pickup_stage_ = AutoPickupStage::kOpenAndCreep;
            stage_start_time_ = now;
            triggerGripDeg(grip_open_cmd_);
          }
        } else {
          aligned_pickup_frames_ = 0;
          // Stable damped rotation + approach law:
          // Pivot first if bearing error > 12 deg (0.21 rad), otherwise smoothly curve in
          double w_cmd = 0.0;
          if (std::abs(bear) > 0.045) {
            w_cmd = std::clamp(nav_cfg_.kp_angular * bear,
                               -nav_cfg_.max_angular_speed_rad_s,
                               nav_cfg_.max_angular_speed_rad_s);
          }
          double v_cmd = 0.0;
          if (std::abs(bear) < 0.24 && dist > follow_standoff_m_ + 0.015) {
            const double align_factor = std::clamp(1.0 - (std::abs(bear) / 0.30), 0.2, 1.0);
            v_cmd = std::clamp(nav_cfg_.kp_linear * (dist - follow_standoff_m_) * align_factor,
                               0.05, nav_cfg_.max_linear_speed_m_s);
          }
          // If reading from persistence memory (briefly occluded), scale down velocity safely
          if (!pickup_det.valid) {
            v_cmd *= 0.5;
            w_cmd *= 0.5;
          }
          mission_out.status_code = 2;
          mission_out.cmd_linear_x_m_s = v_cmd;
          mission_out.cmd_angular_z_rad_s = w_cmd;
          state_name = aruco_det.valid ? "APPROACH_ARUCO" : "APPROACH_TARGET";
        }
      } else if (search_when_lost_) {
        aligned_pickup_frames_ = 0;
        // Stable Step-and-Pause search rotation (0.55s rotate, 0.35s settle pause for crisp ArUco frames)
        const double cycle_pos = std::fmod(now.seconds(), 0.90);
        const double spin_dir = (has_seen_ball_ && last_seen_bearing_rad_ < 0.0) ? -1.0 : 1.0;
        mission_out.status_code = 1;
        mission_out.cmd_angular_z_rad_s = (cycle_pos < 0.55) ? (spin_dir * nav_cfg_.search_spin_rad_s) : 0.0;
        state_name = (cycle_pos < 0.55) ? "SEARCH_TURN" : "SEARCH_SCAN_PAUSE";
      } else {
        mission_out.status_code = 1;
        state_name = "WAIT_ARUCO_CUBE";
      }
    }

    if (pickup_det.valid) {
      publishDetectedObject(pickup_det, target_label, now);
    }

    relbot_interfaces::msg::MissionStatus status_msg;
    status_msg.header.stamp = now;
    status_msg.state = mission_out.status_code;
    status_msg.state_name = state_name;
    status_msg.distance_to_goal_m = mission_out.distance_to_goal_m;
    status_msg.holding_object = (pickup_stage_ == AutoPickupStage::kHoldingCube);
    mission_status_pub_->publish(status_msg);

    geometry_msgs::msg::Twist twist;
    if (enable_motion_) {
      twist.linear.x = mission_out.cmd_linear_x_m_s;
      twist.angular.z = mission_out.cmd_angular_z_rad_s;
    }
    cmd_vel_pub_->publish(twist);

    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "[frame %lu] %s | %s=%s (d=%.2fm, b=%.1fdeg) | cmd=(v=%.2f, w=%.2f) | arm=%.0fdeg grip=%s",
        frame_count_, state_name, target_label.c_str(),
        pickup_det.valid ? "YES" : "no", pickup_det.estimated_distance_m,
        pickup_det.estimated_bearing_rad * kRadToDeg,
        mission_out.cmd_linear_x_m_s, mission_out.cmd_angular_z_rad_s,
        current_arm_deg_, last_grip_label_.c_str());

    // Annotate debug frame
    cv::line(frame_bgr, cv::Point(frame_bgr.cols / 2, 0),
             cv::Point(frame_bgr.cols / 2, frame_bgr.rows - 1),
             cv::Scalar(80, 80, 80), 1);
    const cv::Scalar box_color = aruco_det.valid ? cv::Scalar(255, 230, 0) : cv::Scalar(0, 255, 0);
    drawDetectionBox(frame_bgr, pickup_det, box_color, target_label,
                     aruco_det.valid ? &aruco_det : nullptr);

    char hud[180];
    std::snprintf(hud, sizeof(hud), "%s | v=%.2f w=%.2f | Arm:%.0f Grip:%s",
                  state_name, mission_out.cmd_linear_x_m_s, mission_out.cmd_angular_z_rad_s,
                  current_arm_deg_, last_grip_label_.c_str());
    cv::rectangle(frame_bgr, cv::Point(0, frame_bgr.rows - 20),
                  cv::Point(frame_bgr.cols, frame_bgr.rows), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(frame_bgr, hud, cv::Point(4, frame_bgr.rows - 6),
                cv::FONT_HERSHEY_SIMPLEX, 0.34, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

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
  int capture_width_{640};
  int capture_height_{480};
  bool use_mjpg_{false};
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
  bool search_when_lost_{true};
  double reacquire_timeout_sec_{0.65};
  double min_ball_confidence_{0.42};
  double follow_standoff_m_{0.19};
  int http_port_{8080};
  std::string snapshot_path_;

  // ESP32 Servo & ArUco Pick-Up State
  std::string esp32_serial_port_{"/dev/ttyUSB0"};
  int esp32_fd_{-1};
  std::mutex esp32_mutex_;
  bool auto_pickup_cube_{true};
  double aruco_marker_size_m_{0.030};
  double arm_carry_deg_{20.0};
  double arm_pickup_deg_{-40.0};
  int grip_open_cmd_{0};
  int grip_close_cmd_{180};
  double current_arm_deg_{20.0};
  std::string last_grip_label_{"IDLE"};
  AutoPickupStage pickup_stage_{AutoPickupStage::kSearchOrApproach};
  rclcpp::Time stage_start_time_{0, 0, RCL_ROS_TIME};
  int aligned_pickup_frames_{0};

  double hfov_rad_{1.082};
  double vfov_rad_{0.850};
  uint64_t frame_count_{0};
  int consecutive_ball_frames_{0};
  bool has_seen_ball_{false};
  double smoothed_distance_m_{0.30};
  double smoothed_bearing_rad_{0.0};
  double last_seen_bearing_rad_{0.0};
  RelbotDetectedObjectC last_valid_det_{};
  std::string last_valid_label_{"TARGET"};
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
