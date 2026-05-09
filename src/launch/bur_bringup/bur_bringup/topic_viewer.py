#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TwistStamped, Wrench
from sensor_msgs.msg import Joy, Imu
from nav_msgs.msg import Odometry
from bur_msgs.msg import Command, ThrusterCommand

import tf2_ros
from tf2_ros import TransformException
import math

from rcl_interfaces.msg import SetParametersResult
import numpy as np

def quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    """Convert a unit quaternion (x,y,z,w) to roll/pitch/yaw (radians)."""
    # roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class TopicViewer(Node):
    def __init__(self):
        super().__init__('joy_command')

        # Declare parameters
        self.declare_parameter('command_topic', '/command')
        self.declare_parameter('controller_topic', '/control_effort')
        self.declare_parameter('thruster_topic', '/thruster_command')
        # self.declare_parameter('depth_topic', 'depth_sensor')
        # self.declare_parameter('pose_topic', 'set_pose')
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('update_rate', 100)

        # Timer
        update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value
        self.timer = self.create_timer(1.0 / update_rate, self.timer_callback)

        # Subscriptions
        self.command_sub = self.create_subscription(
            Command,
            self.get_parameter('command_topic').get_parameter_value().string_value,
            self.command_callback,
            10
        )
        self.controller_sub = self.create_subscription(
            Wrench,
            self.get_parameter('controller_topic').get_parameter_value().string_value,
            self.controller_callback,
            10
        )
        self.thruster_sub = self.create_subscription(
            ThrusterCommand,
            self.get_parameter('thruster_topic').get_parameter_value().string_value,
            self.thruster_callback,
            10
        )


        imu_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.get_parameter('imu_topic').get_parameter_value().string_value,
            self.imu_callback,
            imu_qos
        )

        self.command_msg = Command()
        self.controller_msg = Wrench()
        self.thruster_msg =  ThrusterCommand()
        
        self.imu_msg = Imu()
        self.imu_euler_msg = Vector3Stamped()


    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def command_callback(self, msg: Command):
        self.command_msg = msg

    def controller_callback(self, msg: Wrench):
        self.controller_msg = msg

    def thruster_callback(self, msg: ThrusterCommand):
        self.thruster_msg = msg


    def imu_callback(self, msg: Imu):
        self.imu_msg = msg

        # Convert quaternion → Euler and publish
        # Note: tf2 quaternion convention is (x, y, z, w)
        roll, pitch, yaw = self.quat_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        self.imu_euler_msg = Vector3Stamped()
        self.imu_euler_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_euler_msg.header.frame_id = 'base_link'
        self.imu_euler_msg.vector.x = roll
        self.imu_euler_msg.vector.y = pitch
        self.imu_euler_msg.vector.z = yaw
        

    def timer_callback(self):
        def unpack_vector(vec):
            return [vec.x, vec.y, vec.z]
        
        print(' '.join([f'{float(num):10.4f}' for num in unpack_vector(self.command_msg.target_vel.twist.angular)]))


def main(args=None):
    rclpy.init(args=args)
    node = TopicViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
