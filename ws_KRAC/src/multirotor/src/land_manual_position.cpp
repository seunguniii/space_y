#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class LandingTest : public rclcpp::Node {
  public:
    LandingTest() : Node("landing") {
      odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
        curr_odom_ = *msg;
        has_odom_ = true;
      });

      landed_sub_ = this->create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        landed_ = msg->landed;
      });

      desired_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/landing/coordinates", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        desired_x_ = msg->point.x;
        desired_y_ = msg->point.y;
        acc_alt_ = -msg->point.z;
      });

      offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
      trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
      mission_mode_publisher_ = this->create_publisher<std_msgs::msg::String>("/mission_mode", 10);

      auto timer_callback = [this]() -> void {
        if(!has_odom_) {
          RCLCPP_WARN(this->get_logger(), "Waiting for...");
          return;
        }

        if(!armed_ && mission_mode_ != FINISHED) {
          mission_mode_ = LANDING;
          this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
          this->arm();
        }

        publish_offboard_control_mode();

        auto mission_msg = std_msgs::msg::String();
        switch (mission_mode_) {
          case LANDING:
            land();
            mission_msg.data = "LANDING";
            break;

          case FINISHED:
            if(landed_ && armed_) disarm();
            mission_msg.data = "FINISHED";
            if(!armed_) return;
            break;
        }
        mission_mode_publisher_->publish(mission_msg);
        offboard_setpoint_counter_ ++;
      };
      timer_ = this->create_wall_timer(100ms, timer_callback);
    };

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<uint64_t> timestamp_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_mode_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr landed_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr desired_setpoint_sub_;

    px4_msgs::msg::VehicleOdometry curr_odom_;

    enum Mission {
        LANDING,
        FINISHED,
    };

    bool has_odom_ = false;
    bool armed_ = false;
    bool landed_ = false;

    //int hold_counter_ = 0;
    //int HOLD_THRESHOLD = 20;

    float k = 1.0f;

    void arm();
    void disarm();

    void publish_offboard_control_mode();
    void land();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0);

    int offboard_setpoint_counter_ = 0;

    float desired_x_ = 0.0f;
    float desired_y_ = 0.0f;
    float acc_alt_ = 0.0f;

    float low_enough_ = -0.7f;
    float descent_step_ = 1.0f;
    float iter_ratio_ = 0.2f;
    float rad_to_deg = 180/M_PI;

    float nan = std::numeric_limits<float>::quiet_NaN();

    Mission mission_mode_ = LANDING;
};

void LandingTest::arm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
  armed_ = true;
}

void LandingTest::disarm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
  armed_ = false;
}

void LandingTest::publish_offboard_control_mode() {
  OffboardControlMode msg {};
  msg.position = mission_mode_ == LANDING && land_mode_ == 1? false:true;
  msg.velocity = mission_mode_ == LANDING && land_mode_ == 1? true:false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void LandingTest::land() {
  TrajectorySetpoint msg {};

  Eigen::Quaternionf q(curr_odom_.q[0], curr_odom_.q[1], curr_odom_.q[2], curr_odom_.q[3]);

  q.normalize();
  Eigen::Vector3f targetFRD (0, 0, 0);
  iter_ratio_ = log10f(descent_step_*100)*0.1
  float l = -acc_alt_*iter_ratio_;
  if(desired_x_ != 0 || desired_y_ != 0) targetFRD = {desired_y_*l, desired_x_*l, 0};

  //position based landing
  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], acc_alt_);
  Eigen::Vector3f targetNED = current + q*targetFRD;

  msg.position= {targetNED[0], targetNED[1], acc_alt_ + descent_step_};
  break;  

  if(acc_alt_ > low_enough_) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "[LANDING] Low enough at altitude %.3f. Sending land command.", -acc_alt_);
    mission_mode_ = FINISHED;
  }

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void LandingTest::publish_vehicle_command(uint16_t command, float param1, float param2, float param3, float param4) {
  VehicleCommand msg {};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.target_system = 1;
  msg.command = command;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[]) {
  std::cout << "Starting landing test" << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandingTest>());

  rclcpp::shutdown();
  return 0;
}
