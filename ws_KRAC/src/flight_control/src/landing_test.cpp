#include <iostream>
#include <cmath>
#include <chrono>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
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

      this->declare_parameter<float>("descent_step_param", 0.0f);

      offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
      trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

      auto timer_callback = [this]() -> void {
        if(!has_odom_) {
          RCLCPP_WARN(this->get_logger(), "Waiting for...");
          return;
        }

        if(!armed_ && mission_mode_ != FINISHED) {
          this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
          this->arm();
        }

        descent_step_ = (float)this->get_parameter("descent_step_param").as_double();
        iter_ratio_ = log10f(descent_step_*100)*0.1;

        publish_offboard_control_mode();

        switch (mission_mode_) {
          default:
          case FLIGHT:
            publish_trajectory_setpoint();
            break;

          case LANDING:
            land();
            break;

          case FINISHED:
            if(landed_ && armed_) disarm();
            if(!armed_) return;
            break;
        }
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

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr landed_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr desired_setpoint_sub_;

    px4_msgs::msg::VehicleOdometry curr_odom_;

    enum Mission {
      FLIGHT,
      LANDING,
      FINISHED,
    };

    enum FlightMode {
      MULTIROTOR = 3,
      FIXED_WING = 4,
    };

    bool has_odom_ = false;
    bool armed_ = false;
    bool landed_ = false;

    std::vector<std::array<float, 3>> waypoints_ = {
      {0, 0, -10},
      {200, 0, -10},
      {300, 100, -10},
      {300, -100, -10},
      {200, 0, -10},
      {5, 3, -10}
    };

    size_t wp_idx_ = 0;

    int hold_counter_ = 0;
    int HOLD_THRESHOLD = 20;

    float k = 1.0f;

    void arm();
    void disarm();

    void publish_offboard_control_mode();
    void transition(FlightMode mode);
    void publish_trajectory_setpoint();
    void land();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

    void publish_vehicle_command(int command, float value);

    int offboard_setpoint_counter_ = 0;

    float desired_x_ = 0.0f;
    float desired_y_ = 0.0f;
    float acc_alt_ = 0.0f;

    float low_enough_ = -0.5f;
    float descent_step_ = 1.0f;
    float iter_ratio_ = 0.2f;

    FlightMode flight_mode_ = MULTIROTOR;
    Mission mission_mode_ = FLIGHT;
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

void LandingTest::transition(LandingTest::FlightMode mode){
  if (mode == flight_mode_) return;

  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, static_cast<float>(mode));
  flight_mode_ = mode;
  RCLCPP_INFO(this->get_logger(), "[Transition] Transitioning from flight mode %s to %s.",
                                   flight_mode_ == FIXED_WING? "MULTIROTOR":"FIXED_WING",
                                   flight_mode_ == MULTIROTOR? "MULTIROTOR":"FIXED_WING");
}

void LandingTest::publish_offboard_control_mode() {
  OffboardControlMode msg {};
  msg.position = true;
  msg.velocity = flight_mode_ == FIXED_WING? true:false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(msg);
}

void LandingTest::publish_trajectory_setpoint() {
  if(curr_odom_.timestamp == 0) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  TrajectorySetpoint msg {};
  auto &wp = waypoints_[wp_idx_];

  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  Eigen::Vector3f target(wp[0], wp[1], wp[2]);
  Eigen::Vector3f to_wp = target - current;
  float dist = to_wp.norm();
  Eigen::Vector3f dir = to_wp/dist;

  if(armed_) {
    switch(flight_mode_) {
      case MULTIROTOR:
        msg.position = {wp[0], wp[1], wp[2]};

        if(dist < 3.0f) {
          hold_counter_++;
          if(hold_counter_ > HOLD_THRESHOLD){
            hold_counter_ = 0;
            wp_idx_++;
            RCLCPP_INFO(this->get_logger(), "[MULTIROTOR] Reached waypoint %ld. Heading waypoint %ld.", wp_idx_-1, wp_idx_);
            transition(FIXED_WING);
          }
        }
        break;

      case FIXED_WING:
        msg.position = {wp[0], wp[1], wp[2]};
        msg.velocity = {k*dir.x(), k*dir.y(), 0.0f};

        if(dist < 10.0f){
          wp_idx_++;
          RCLCPP_INFO(this->get_logger(), "[FIXED_WING] Reached waypoint %ld. Heading waypoint %ld.", wp_idx_-1, wp_idx_);
        }
        if(wp_idx_ >= waypoints_.size()){
          transition(MULTIROTOR);
          mission_mode_ = LANDING;
          hold_counter_= 0;
        }

      default:
        break;
  }
}

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void LandingTest::land() {
  TrajectorySetpoint msg {};

  Eigen::Quaternionf q(curr_odom_.q[0], curr_odom_.q[1], curr_odom_.q[2], curr_odom_.q[3]);
  q.normalize();

  //acc_alt_ = curr_odom_.position[2];

  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], acc_alt_);
  float k = -acc_alt_*iter_ratio_;
  Eigen::Vector3f targetFRD (0, 0, 0);
  if(desired_x_ != 0 || desired_y_ != 0) targetFRD = {desired_y_*k, desired_x_*k, 0};
  Eigen::Vector3f targetNED = current + q*targetFRD;

  if(desired_x_ == 0 && desired_y_ == 0) targetNED = {waypoints_[waypoints_.size() - 1][0], waypoints_[waypoints_.size() - 1][1], acc_alt_};
  if(acc_alt_ > low_enough_) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(), "[Landing] Low enough at altitude %.3f. Sending land command.", -acc_alt_);
    mission_mode_ = FINISHED;
  }

  msg.position = {targetNED[0], targetNED[1], acc_alt_ + descent_step_};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void LandingTest::publish_vehicle_command(uint16_t command, float param1, float param2) {
  VehicleCommand msg {};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
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
