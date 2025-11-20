#include <iostream>
#include <chrono>
#include <vector>
#include <stdint.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class FlightTest : public rclcpp::Node {
  public:
    FlightTest() : Node("FlightTest") {
      odom_sub_ = this->create_subscription<VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
                           [this](const VehicleOdometry::SharedPtr msg) {
                           curr_odom_ = *msg; has_odom_ = true;});

      offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
      trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

      auto timer_callback = [this]() -> void {
        if(!has_odom_){
          RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
          return;
        }

        if (!armed_ && flight_mode_ != LANDED) {
          this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
          this->arm();
        }

        publish_offboard_control_mode();
        publish_trajectory_setpoint();

        offboard_setpoint_counter_++;
      };
      timer_ = this->create_wall_timer(100ms, timer_callback);
    };

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

    rclcpp::Subscription<VehicleOdometry>::SharedPtr odom_sub_;

    VehicleOdometry curr_odom_;

    enum FlightMode {
        MULTIROTOR = 3,
        FIXED_WING = 4,
        LANDED
    };

    FlightMode flight_mode_ = MULTIROTOR;

    std::vector<std::array<float,3>> waypoints_ = {
      {0.0f, 0.0f, -25.0f},
      {100.0f, 0.0f, -25.0f},
      {200.0f, 100.0f, -25.0f},
      {200.0f, -100.0f, -25.0f},
      {100.0f, 0.0f, -25.0f},
      {0.0f, 0.0f, -25.0f},
      {0.0f, 0.0f, 0.0f}
    };

    uint64_t offboard_setpoint_counter_ {0};
    size_t wp_idx_ {0};

    bool has_odom_ = false;
    bool armed_ = false;

    int hold_counter_ = 0;
    const int HOLD_THRESHOLD = 20;

    void arm();
    void disarm();

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void transition(FlightMode mode = MULTIROTOR);

    void publish_vehicle_command(int command, float value);
    float k = 1;
};

void FlightTest::arm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
  armed_ = true;
}

void FlightTest::disarm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
  armed_ = false;
}

void FlightTest::publish_offboard_control_mode() {
  OffboardControlMode msg {};
  msg.position = true;
  msg.velocity = flight_mode_ == MULTIROTOR? false:true;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void FlightTest::publish_trajectory_setpoint() {
  if (curr_odom_.timestamp == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  TrajectorySetpoint msg {};

  auto &wp = waypoints_[wp_idx_];

  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  Eigen::Vector3f target(wp[0], wp[1], wp[2]);
  Eigen::Vector3f to_wp = target - current;
  float dist_to_wp = to_wp.norm();
  Eigen::Vector3f direction_v(to_wp.x()/dist_to_wp, to_wp.y()/dist_to_wp, to_wp.z()/dist_to_wp);

  if (flight_mode_ == MULTIROTOR && armed_) {
    msg.position = {wp[0], wp[1], wp[2]};

    if (dist_to_wp < 3.0f) {
      hold_counter_++;
      if (hold_counter_ > HOLD_THRESHOLD) {
        RCLCPP_INFO(this->get_logger(), "[MULTIROTOR] Heading to waypoint %ld", wp_idx_ + 1);
        hold_counter_ = 0;
        wp_idx_++;
        if(wp_idx_ == 1) transition(FIXED_WING);
        if(wp_idx_ >= waypoints_.size()) {
          RCLCPP_INFO(this->get_logger(), "[LANDING] Sending land command");
          publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
          flight_mode_ = LANDED;
          disarm();
        }
      }
    }
  }

  else if (flight_mode_ == FIXED_WING) {
    msg.position = {wp[0], wp[1], wp[2]};
    msg.velocity = {k*direction_v.x(), k*direction_v.y(), 0};

    if (dist_to_wp < 10.0f) {
      wp_idx_++;
      RCLCPP_INFO(this->get_logger(), "[FIXED_WING] Heading to waypoint %ld", wp_idx_);
    }

    if (wp_idx_ + 2> waypoints_.size()) transition(MULTIROTOR);
  }

  else if (flight_mode_ == LANDED) return;

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void FlightTest::publish_vehicle_command(uint16_t command, float param1, float param2) {
  VehicleCommand msg {};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

void FlightTest::transition(FlightMode mode) {
  if (mode == flight_mode_) {
    RCLCPP_INFO(this->get_logger(), "[TRANSITION] Already in desired flight mode, no command sent.");
    return;
  }

  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, static_cast<float>(mode));
  std::string mode_str = (mode == FIXED_WING) ? "Fixed-Wing" : "Multicopter";
  RCLCPP_INFO(this->get_logger(), "[TRANSITION] Sent VTOL transition command: %s", mode_str.c_str());
  flight_mode_ = mode;
}

int main(int argc, char *argv[]) {
  std::cout << "Starting offboard control node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FlightTest>());

  rclcpp::shutdown();
  return 0;
}
