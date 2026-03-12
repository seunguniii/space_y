import math 
import sys 

import rclpy 
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode

class FlightTest(Node):
    def __init__(self):
        super().__init__('flight_test')

        self.offboard_enabled = False
        self.armed = VehicleStatus.ARMING_STATE_DISARMED
        
        self.position = [0.0, 0.0, 0.0]
        self.target = [
            [0.0, 0.0, -10.0],
            [5.0, 0.0, -10.0],
            [5.0, 5.0, -10.0],
            [5.0, -5.0, -10.0],
            [5.0, 0.0, -10.0],
            [0.0, 0.0, -10.0]
        ]

        self.target_index = 0
        self.reached_target_counter = 0
        self.distance_threshold = 2.0
        self.time_threshold = 300 #3초 머무르기 

        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, 
            depth=0
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        self.subscriber_odometry = self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self.callback_odometry,
            qos_profile_sub 
        )

        self.subscriber_vehicle_status = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.callback_vehicle_status,
            qos_profile_sub
        )

        self.subscriber_vehicle_control_mode = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.callback_vehicle_control_mode, 
            qos_profile_sub
        )
        
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile_pub
        )

        self.publisher_trajectory_setpoint = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile_pub
        )

        self.publisher_vehicle_command = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile_pub
        )

        timer_period = 0.01 
        self.timer = self.create_timer(timer_period, self.callback_cmdloop)
                                                                
    def callback_odometry(self, msg):
        self.position = msg.position  
    
    def callback_vehicle_status(self, msg):
        self.armed = msg.arming_state  

    def callback_vehicle_control_mode(self, msg):
        self.offboard_enabled = msg.flag_control_offboard_enabled 

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False 
        msg.acceleration = False 
        msg.attitude = False 
        msg.body_rate = False 
        msg.thrust_and_torque = False 
        msg.direct_actuator = False 
        self.publisher_offboard_mode.publish(msg)

    def publish_vehicle_command(self, command, param1, param2):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command 
        msg.from_external = True
        self.publisher_vehicle_command.publish(msg)

    def publish_arm_command(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        print("Arm command send") 
    
    def publish_disarm_command(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        print("Disarm command send") 
    
    def publish_trajectory_setpoint(self, p):
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = p 
        self.publisher_trajectory_setpoint.publish(msg)
    
    def close_enough(self):
        distance_to_target = math.dist(self.position, self.target[self.target_index])
        if distance_to_target < self.distance_threshold:
            return True
        else:
            return False
        
    def callback_cmdloop(self):
        if self.armed == VehicleStatus.ARMING_STATE_DISARMED:
            self.publish_arm_command()
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

        self.publish_offboard_mode()

        self.publish_trajectory_setpoint(self.target[self.target_index])

        if self.close_enough():
            self.reached_target_counter += 1.0
            if self.reached_target_counter > self.time_threshold:
                if self.target_index < len(self.target) - 1:
                    self.target_index += 1 
                    self.reached_target_counter = 0.0
                else: #land 추가 
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0)
                    self.get_logger().info("LAND COMMAND SEND")
                    self.timer.cancel()
                    sys.exit(0)
                
def main(args=None):
    rclpy.init(args=args)
    flight_test = FlightTest()
    rclpy.spin(flight_test)
    flight_test.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
