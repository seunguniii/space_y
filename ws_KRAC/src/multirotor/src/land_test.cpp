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
#include <limits>
#include "px4_msgs/msg/vehicle_status.hpp"

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
      
      vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
  "/fmu/out/vehicle_status_v1",
  rclcpp::SensorDataQoS(),
  [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
    vehicle_status_ = *msg;
    has_vehicle_status_ = true;

    armed_ = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
  }
);

      desired_setpoint_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/landing/coordinates", 10,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        desired_x_ = msg->point.x;
        desired_y_ = msg->point.y;
        acc_alt_ = -msg->point.z;
      });

      //0: start from manual setpoint  1: start from setpoint by param
      this->declare_parameter<int>("start_param", 0);
      this->declare_parameter<float>("start_x_param", 0.0f);
      this->declare_parameter<float>("start_y_param", 0.0f);
      this->declare_parameter<float>("start_z_param", 0.0f);
      this->declare_parameter<int>("lost_abort_",700);
      this->declare_parameter<float>("kp_xy_",1.2f);   // p 제어항
      this->declare_parameter<float>("max_xy_",0.6f); // [m/s]
      this->declare_parameter<float>("tol_m_",0.8f); // [m] 정렬 허용 오차(예: 12cm)
      this->declare_parameter<int>("align_need_",5);  
      this->declare_parameter<float>("kd_xy_", 0.08f); // d 제어항
      
      offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
      trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
      vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
      mission_mode_publisher_ = this->create_publisher<std_msgs::msg::String>("/mission_mode", 10);

auto timer_callback = [this]() -> void {
  if(!has_odom_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for odometry...");
    return;
  }

  if(!has_vehicle_status_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for vehicle status...");
    return;
  }

  start_mode_ = this->get_parameter("start_param").as_int();

  if(start_mode_ == 0) {
    mission_mode_ = LANDING;
  }

  start_x = (float)this->get_parameter("start_x_param").as_double();
  start_y = (float)this->get_parameter("start_y_param").as_double();
  start_z = - (float)this->get_parameter("start_z_param").as_double();
  kp_xy_ = (float)this->get_parameter("kp_xy_").as_double();
  max_xy_ = (float)this->get_parameter("max_xy_").as_double();
  tol_m_  = (float)this->get_parameter("tol_m_").as_double();
  align_need_ = this->get_parameter("align_need_").as_int();
  lost_abort_ = this->get_parameter("lost_abort_").as_int();
  kd_xy_ = (float)this->get_parameter("kd_xy_").as_double();

  publish_offboard_control_mode();

  auto mission_msg = std_msgs::msg::String();

  switch (mission_mode_) {
    default:
    case FLIGHT:
      publish_trajectory_setpoint();
      mission_msg.data = "FLIGHT";
      break;

    case LANDING:
      land();
      mission_msg.data = "LANDING";
      break;

    case FINISHED:
      if(landed_ && armed_) {
        disarm();
      }

      mission_msg.data = "FINISHED";
      mission_mode_publisher_->publish(mission_msg);
      return;
  }

  mission_mode_publisher_->publish(mission_msg);

  // 1) 먼저 setpoint stream 확보
  if (!offboard_requested_) {
    preflight_setpoint_count_++;

    if (preflight_setpoint_count_ > 20) {
      RCLCPP_INFO(this->get_logger(), "Requesting OFFBOARD mode");
      publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      offboard_requested_ = true;
    }

    return;
  }

  // 2) Offboard 요청 후 arm 요청
  if (!armed_ && !arm_requested_) {
    RCLCPP_INFO(this->get_logger(), "Requesting ARM");
    arm();
    arm_requested_ = true;
    return;
  }

  // 3) arm 요청했는데 아직 실제 arm 안 됨
  if (arm_requested_ && !armed_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      1000,
      "Waiting for actual ARM state from PX4..."
    );
    return;
  }

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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_mode_publisher_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr landed_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr desired_setpoint_sub_;
    rclcpp::Time prev_ctrl_time_;
    px4_msgs::msg::VehicleOdometry curr_odom_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;

    enum Mission {
        FLIGHT,
        LANDING,
        FINISHED,
    };

    bool has_odom_ = false;
    bool armed_ = false;
    bool landed_ = false;

    int hold_counter_ = 0;
    int HOLD_THRESHOLD = 20;

    void arm();
    void disarm();

    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();
    void land();
    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, float param3 = 0.0, float param4 = 0.0);

    int offboard_setpoint_counter_ = 0;

    float desired_x_ = 0.0f;
    float desired_y_ = 0.0f;
    float acc_alt_ = 0.0f;

    float low_enough_ = -0.7f;
    float rad_to_deg = 180/M_PI;

    float start_x = 0.0f;
    float start_y = 0.0f;
    float start_z = 0.0f;
    float kp_xy_ = 1.2f; // 착륙시 P 계수
    float max_xy_ = 0.4f; // max 수평방향 속도
    float tol_m_ = 1.2f; // 허용가능한 위치 오차
    int align_need_ = 5; // 몇 tick 연속 정렬이면 하강 허용
    int lost_abort_ = 30; // abort 하는 경계값 (tick)
    bool use_q_inverse_ = false; // 무시
    int lost_count_ = 0; // dx, dy nan 값 나오는 틱 카운트

    float nan = std::numeric_limits<float>::quiet_NaN();

    int start_mode_ = 0;
    int land_mode_ = 1;
    Mission mission_mode_ = FLIGHT;

    float hold_x_ = 0.0f;
    float hold_y_ = 0.0f;
    float hold_z_ = 0.0f; 

    float kd_xy_ = 0.08f;

    float prev_ex_ = 0.0f;
    float prev_ey_ = 0.0f;
    bool has_prev_error_ = false; // d 제어에 필요한 변수
    px4_msgs::msg::VehicleStatus vehicle_status_;
    bool has_vehicle_status_ = false;
    bool arm_requested_ = false;
    bool offboard_requested_ = false;
    int preflight_setpoint_count_ = 0;
};

static float clampf(float v, float lim) {
  if (v > lim) return lim;
  if (v < -lim) return -lim;
  return v;
}

void LandingTest::arm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

void LandingTest::disarm() {
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void LandingTest::publish_offboard_control_mode() {
  OffboardControlMode msg {};
   msg.position = mission_mode_ == LANDING && land_mode_ == 1 ? false : true;
  msg.velocity = mission_mode_ == LANDING && land_mode_ == 1? true:false;
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

  Eigen::Vector3f current(curr_odom_.position[0], curr_odom_.position[1], curr_odom_.position[2]);
  Eigen::Vector3f target(start_x, start_y, start_z);
  Eigen::Vector3f to_wp = target - current;
  float dist = to_wp.norm();

  if(armed_) {
    msg.position = {target[0], target[1], target[2]};

    if(dist < 3.0f) {
      hold_counter_++;
      if(hold_counter_ > HOLD_THRESHOLD) {
        hold_counter_ = 0;
        mission_mode_ = LANDING;
        RCLCPP_INFO(this->get_logger(), "[LANDING] Initiating landing sequence");
        return;
      }
    }
  }

  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}

void LandingTest::land() {
  TrajectorySetpoint msg{};

  const float alt_m = -acc_alt_;  // python z(+) -> 여기 acc_alt_는 -z 형태

  const bool valid_xy = std::isfinite(desired_x_) && std::isfinite(desired_y_);
  const bool aligned  = valid_xy &&
                        (std::fabs(desired_x_) < tol_m_) &&
                        (std::fabs(desired_y_) < tol_m_);

  // lost / align counters
  if (!valid_xy) {
    lost_count_++;
    hold_counter_ = 0;
  } else {
    lost_count_ = 0;
    hold_counter_ = aligned ? (hold_counter_ + 1) : 0;
  }

  if (lost_count_ > lost_abort_) {
    RCLCPP_WARN(
      this->get_logger(),
      "[LANDING] target lost too long -> switch PX4 to POSITION mode"
    );

    publish_vehicle_command(
      VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
      1.0f,
      3.0f
    );

    mission_mode_ = FINISHED;
    return;
  }

  // --- P, D control in BODY(FRD) ---
  // desired_y_: 전방(+) [m], desired_x_: 우측(+) [m] 라는 정의로 사용
  float v_fwd = 0.0f;
  float v_rgt = 0.0f;
  float deadband = 0.05f;

  if (valid_xy) {
    float ex = desired_x_;
    float ey = desired_y_;

    if (std::fabs(ex) < deadband) ex = 0.0f;
    if (std::fabs(ey) < deadband) ey = 0.0f;

    auto now = this->get_clock()->now();
    float dt = 0.1f;

    if (has_prev_error_) {
      dt = (now - prev_ctrl_time_).seconds();
      if (dt <= 0.001f || dt > 1.0f) {
        dt = 0.1f;
    }
  }
  float dex_dt = 0.0f;
  float dey_dt = 0.0f;

  if (has_prev_error_) {
    dex_dt = (ex - prev_ex_) / dt;
    dey_dt = (ey - prev_ey_) / dt;
  }

  v_rgt = clampf(kp_xy_ * ex + kd_xy_ * dex_dt, max_xy_);
  v_fwd = clampf(kp_xy_ * ey + kd_xy_ * dey_dt, max_xy_);

  prev_ex_ = ex;
  prev_ey_ = ey;
  prev_ctrl_time_ = now;
  has_prev_error_ = true;
} else {
  has_prev_error_ = false;
}


  // --- rotate BODY(FRD) -> NED ---
  Eigen::Quaternionf q(curr_odom_.q[0], curr_odom_.q[1], curr_odom_.q[2], curr_odom_.q[3]);
  q.normalize();

  Eigen::Vector3f v_body(v_fwd, v_rgt, 0.0f);
  Eigen::Vector3f v_ned = use_q_inverse_ ? (q.conjugate() * v_body) : (q * v_body);

  // --- descent gating (정렬될 때만 하강) ---
  float vz_down = 0.0f;  // 너 기존 코드 관례대로 +가 하강이라고 가정
  if (valid_xy && hold_counter_ >= align_need_) {
    if (alt_m > 2.0f)      vz_down = 0.30f;
    else if (alt_m > 0.8f) vz_down = 0.20f;
    else                   vz_down = 0.10f;
  }
  
  RCLCPP_INFO(this->get_logger(),
  "valid=%d aligned=%d hold=%d/%d dx=%.3f dy=%.3f tol=%.3f vz=%.3f",
  valid_xy, aligned, hold_counter_, align_need_,
  desired_x_, desired_y_, tol_m_, vz_down);

  // publish velocity setpoint (한 번만)
  msg.position = {nan, nan, nan};
  msg.velocity = {v_ned[0], v_ned[1], vz_down};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);

  // final NAV_LAND only when aligned & low enough
  if (valid_xy && hold_counter_ >= align_need_ && acc_alt_ > low_enough_) {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(this->get_logger(),
                "[LANDING] aligned & low enough (alt=%.2f m). NAV_LAND.", alt_m);
    mission_mode_ = FINISHED;
  }
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
