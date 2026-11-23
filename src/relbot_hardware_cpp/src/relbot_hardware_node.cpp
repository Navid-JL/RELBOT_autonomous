#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "relbot_hardware_cpp/manipulator_sequencer.hpp"
#include "relbot_hardware_cpp/relbot_base_driver.hpp"
#include "relbot_hardware_cpp/serial_mcu_bridge.hpp"
#include "relbot_interfaces/msg/arm_command.hpp"
#include "relbot_interfaces/msg/manipulator_state.hpp"
#include "relbot_interfaces/srv/execute_pick_place.hpp"

namespace relbot::hw {

class RelbotHardwareNode : public rclcpp::Node {
 public:
  RelbotHardwareNode() : Node("relbot_hardware_node") {
    // Declare hardware parameters
    const auto serial_port = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    const auto baud_rate = this->declare_parameter<int>("baud_rate", 115200);
    const auto sim_fallback = this->declare_parameter<bool>("simulate_if_unavailable", true);
    const auto wheel_radius = this->declare_parameter<double>("wheel_radius_m", 0.050);
    const auto track_width = this->declare_parameter<double>("track_width_m", 0.210);
    const auto max_wheel_speed = this->declare_parameter<double>("max_wheel_rad_s", 12.0);

    mcu_bridge_ = std::make_unique<SerialMcuBridge>(serial_port, baud_rate, sim_fallback);
    base_driver_ = std::make_unique<RelbotBaseDriver>(wheel_radius, track_width, max_wheel_speed);
    sequencer_ = std::make_unique<ManipulatorSequencer>();

    if (!mcu_bridge_->connect()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to connect to MCU serial port %s", serial_port.c_str());
    } else if (mcu_bridge_->isSimulationMode()) {
      RCLCPP_WARN(this->get_logger(),
                  "Serial port %s unavailable; running Microcontroller Bridge in SIMULATION mode",
                  serial_port.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Connected to MCU serial bridge on %s @ %d baud",
                  serial_port.c_str(), baud_rate);
    }

    // Subscribers from Rust autonomous navigation node
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          latest_linear_m_s_ = msg->linear.x;
          latest_angular_rad_s_ = msg->angular.z;
        });

    arm_cmd_sub_ = this->create_subscription<relbot_interfaces::msg::ArmCommand>(
        "/arm/command", 10,
        [this](const relbot_interfaces::msg::ArmCommand::SharedPtr msg) {
          // Direct manual override cancels active macro sequence
          sequencer_->startSequence(ManipulatorSequencer::SequenceType::kNone);
          direct_cmd_.arm_angle_rad = msg->arm_angle_rad;
          direct_cmd_.arm_velocity_rad_s =
              (msg->arm_velocity_rad_s > 0.0) ? msg->arm_velocity_rad_s : 1.2;
          direct_cmd_.gripper_angle_deg = msg->gripper_angle_deg;
          direct_cmd_.torque_enable = msg->torque_enable;
          mcu_bridge_->sendCommand(direct_cmd_);
        });

    // Publishers to RELBot low-level motor drivers (XRF2 / FPGA setpoint topics) & Rust stack
    left_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/input/left_motor/setpoint_vel", 10);
    right_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>(
        "/input/right_motor/setpoint_vel", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    manip_state_pub_ = this->create_publisher<relbot_interfaces::msg::ManipulatorState>(
        "/arm/state", 10);

    // Service for atomic pick-up / release macro trigger
    pick_place_srv_ = this->create_service<relbot_interfaces::srv::ExecutePickPlace>(
        "/arm/execute_pick_place",
        [this](const std::shared_ptr<relbot_interfaces::srv::ExecutePickPlace::Request> req,
               std::shared_ptr<relbot_interfaces::srv::ExecutePickPlace::Response> res) {
          if (req->action == relbot_interfaces::srv::ExecutePickPlace::Request::ACTION_PICKUP) {
            sequencer_->startSequence(ManipulatorSequencer::SequenceType::kPickup);
            res->success = true;
            res->message = "Started 1-DOF XL430 + MG996R PICKUP sequence";
          } else if (req->action == relbot_interfaces::srv::ExecutePickPlace::Request::ACTION_RELEASE) {
            sequencer_->startSequence(ManipulatorSequencer::SequenceType::kRelease);
            res->success = true;
            res->message = "Started 1-DOF XL430 + MG996R RELEASE sequence";
          } else {
            sequencer_->startSequence(ManipulatorSequencer::SequenceType::kHome);
            res->success = true;
            res->message = "Started HOMING sequence";
          }
        });

    // 50 Hz hardware control & telemetry loop
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&RelbotHardwareNode::controlLoop, this));
  }

 private:
  void controlLoop() {
    constexpr double kDtSec = 0.02;

    // 1. Convert body twist to RELBot left/right wheel angular velocities
    const auto wheels = base_driver_->twistToWheelVelocities(latest_linear_m_s_, latest_angular_rad_s_);

    std_msgs::msg::Float64 left_msg;
    left_msg.data = wheels.left_rad_s;
    left_motor_pub_->publish(left_msg);

    std_msgs::msg::Float64 right_msg;
    right_msg.data = wheels.right_rad_s;
    right_motor_pub_->publish(right_msg);

    // 2. Update odometry and publish /odom
    const auto odom_pose = base_driver_->updateOdometry(wheels.left_rad_s, wheels.right_rad_s, kDtSec);
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose.position.x = odom_pose.x_m;
    odom_msg.pose.pose.position.y = odom_pose.y_m;
    odom_msg.pose.pose.orientation.z = std::sin(odom_pose.yaw_rad * 0.5);
    odom_msg.pose.pose.orientation.w = std::cos(odom_pose.yaw_rad * 0.5);
    odom_msg.twist.twist.linear.x = odom_pose.linear_vel_m_s;
    odom_msg.twist.twist.angular.z = odom_pose.angular_vel_rad_s;
    odom_pub_->publish(odom_msg);

    // 3. Poll microcontroller telemetry and update pick/place sequence if active
    mcu_bridge_->pollTelemetry(latest_telemetry_, kDtSec);
    if (sequencer_->isBusy()) {
      const auto cmd = sequencer_->update(latest_telemetry_, kDtSec);
      mcu_bridge_->sendCommand(cmd);
    }

    // 4. Publish manipulator telemetry to Rust navigation FSM
    relbot_interfaces::msg::ManipulatorState state_msg;
    state_msg.header.stamp = this->now();
    state_msg.header.frame_id = "arm_link";
    state_msg.current_arm_angle_rad = latest_telemetry_.current_arm_angle_rad;
    state_msg.current_arm_velocity_rad_s = latest_telemetry_.current_arm_velocity_rad_s;
    state_msg.arm_load = latest_telemetry_.arm_load_ma;
    state_msg.current_gripper_angle_deg = latest_telemetry_.current_gripper_angle_deg;
    state_msg.target_reached = !sequencer_->isBusy() && latest_telemetry_.target_reached;
    state_msg.object_grasped = latest_telemetry_.object_grasped;
    state_msg.error_code = latest_telemetry_.error_code;
    state_msg.status_message = latest_telemetry_.status_message;
    manip_state_pub_->publish(state_msg);
  }

  std::unique_ptr<SerialMcuBridge> mcu_bridge_;
  std::unique_ptr<RelbotBaseDriver> base_driver_;
  std::unique_ptr<ManipulatorSequencer> sequencer_;

  double latest_linear_m_s_{0.0};
  double latest_angular_rad_s_{0.0};
  ManipulatorCommand direct_cmd_{};
  ManipulatorTelemetry latest_telemetry_{};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<relbot_interfaces::msg::ArmCommand>::SharedPtr arm_cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<relbot_interfaces::msg::ManipulatorState>::SharedPtr manip_state_pub_;
  rclcpp::Service<relbot_interfaces::srv::ExecutePickPlace>::SharedPtr pick_place_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace relbot::hw

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<relbot::hw::RelbotHardwareNode>());
  rclcpp::shutdown();
  return 0;
}
