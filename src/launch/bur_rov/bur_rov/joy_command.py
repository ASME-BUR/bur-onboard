#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TwistStamped
from sensor_msgs.msg import Joy, Imu
from nav_msgs.msg import Odometry
from bur_msgs.msg import Command

import tf2_ros
from tf2_ros import TransformException
import math

from rcl_interfaces.msg import SetParametersResult
import numpy as np


class JoyCommand(Node):
    def __init__(self):
        super().__init__('joy_command')

        # Declare parameters
        self.declare_parameter('cmd_pub_topic', 'command')
        self.declare_parameter('twist_topic', 'set_twist')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('depth_topic', 'depth_sensor')
        self.declare_parameter('pose_topic', 'set_pose')
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('multiplier', 0.5)
        self.declare_parameter('using_ekf', False)
        self.declare_parameter('using_joy', True)
        self.declare_parameter('publish_rate', 100)

        # Axis mapping parameters
        axis_defaults = {
            'linear_x':  1,
            'linear_y':  0,
            'linear_z':  2,
            'angular_x': 0,
            'angular_y': 0,
            'angular_z': 4,
        }
        for key, val in axis_defaults.items():
            self.declare_parameter(f'axis_mapping.{key}', val)

        # Publishers
        self.cmd_pub = self.create_publisher(
            Command,
            self.get_parameter('cmd_pub_topic').get_parameter_value().string_value,
            10
        )
        self.imu_euler_pub = self.create_publisher(Vector3Stamped, 'imu_euler', 10)
        self.joy_euler_pub = self.create_publisher(Vector3Stamped, 'joy_euler', 10)

        # Timer
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # Subscriptions
        self.joy_sub = self.create_subscription(
            Joy,
            self.get_parameter('joy_topic').get_parameter_value().string_value,
            self.joy_callback,
            10
        )
        self.twist_sub = self.create_subscription(
            TwistStamped,
            self.get_parameter('twist_topic').get_parameter_value().string_value,
            self.twist_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )
        self.pose_sub = self.create_subscription(
            Odometry,
            'next_waypoint',
            self.des_pose_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter('depth_topic').get_parameter_value().string_value,
            self.depth_callback,
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

        # State
        self.output = Command()
        self.new_params = False
        self.v_0 = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.prev_time = self.get_clock().now()
        self.axis_mapping: dict[str, int] = {}

        self.set_constants()

        # Parameter change callback
        self.add_on_set_parameters_callback(self.parameters_callback)

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    def set_constants(self):
        self.using_joy = self.get_parameter('using_joy').get_parameter_value().bool_value
        self.using_ekf = self.get_parameter('using_ekf').get_parameter_value().bool_value
        self.multiplier = self.get_parameter('multiplier').get_parameter_value().double_value

        keys = ['linear_x', 'linear_y', 'linear_z', 'angular_x', 'angular_y', 'angular_z']
        for key in keys:
            self.axis_mapping[key] = (
                self.get_parameter(f'axis_mapping.{key}')
                .get_parameter_value().integer_value
            )

        self.get_logger().info(f"angular_z axis: {self.axis_mapping['angular_z']}")
        self.new_params = False

    def parameters_callback(self, parameters):
        for param in parameters:
            self.get_logger().info(f'{param.name}')
            self.get_logger().info(f'{param.type_}')
            self.get_logger().info(f'{param.value}')
        self.new_params = True
        result = SetParametersResult()
        result.successful = True
        result.reason = 'success'
        return result

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------

    def joy_callback(self, msg: Joy):
        if not self.using_joy or not msg.axes:
            return

        now = self.get_clock().now().to_msg()
        self.output.header.stamp = now
        self.output.header.frame_id = 'odom'
        self.output.target_vel.header.stamp = now
        self.output.target_vel.header.frame_id = 'odom'

        def get_axis(mapping: int) -> float:
            if mapping < 0 or abs(mapping) >= len(msg.axes):
                return 0.0
            return float(msg.axes[mapping])

        m = self.multiplier
        am = self.axis_mapping
        self.output.target_vel.twist.linear.x  =  m * -get_axis(am['linear_x'])
        self.output.target_vel.twist.linear.y  =  m * -get_axis(am['linear_y'])
        self.output.target_vel.twist.linear.z  =  m *  get_axis(am['linear_z'])
        self.output.target_vel.twist.angular.x =  m *  get_axis(am['angular_x'])
        self.output.target_vel.twist.angular.y =  m * -get_axis(am['angular_y'])
        self.output.target_vel.twist.angular.z =  m * -get_axis(am['angular_z'])

    def depth_callback(self, msg: PoseWithCovarianceStamped):
        self.output.current_pos.header.stamp = self.get_clock().now().to_msg()
        self.output.current_pos.pose.position.z = msg.pose.pose.position.z

    def twist_callback(self, msg: TwistStamped):
        if not self.using_joy:
            self.output.target_vel = msg

    def des_pose_callback(self, msg: Odometry):
        self.output.target_pos.pose.position.z = msg.pose.pose.position.z

    def imu_callback(self, msg: Imu):
        if not self.using_ekf:
            self.output.current_pos.header.stamp = self.get_clock().now().to_msg()
            self.output.current_pos.header.frame_id = 'imu'
            self.output.current_pos.header = msg.header
            self.output.current_pos.pose.orientation = msg.orientation

            self.output.current_vel.header.frame_id = 'imu'
            self.output.current_vel.twist.angular.x = msg.angular_velocity.x
            self.output.current_vel.twist.angular.y = msg.angular_velocity.y
            self.output.current_vel.twist.angular.z = msg.angular_velocity.z

        # Convert quaternion → Euler and publish
        # Note: tf2 quaternion convention is (x, y, z, w)
        roll, pitch, yaw = self._quat_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        imu_euler_msg = Vector3Stamped()
        imu_euler_msg.header.stamp = self.get_clock().now().to_msg()
        imu_euler_msg.header.frame_id = 'base_link'
        imu_euler_msg.vector.x = roll
        imu_euler_msg.vector.y = pitch
        imu_euler_msg.vector.z = yaw
        self.imu_euler_pub.publish(imu_euler_msg)

    def odom_callback(self, msg: Odometry):
        if self.using_ekf:
            self.output.header.stamp = self.get_clock().now().to_msg()
            self.output.header.frame_id = 'base_link'
            self.output.current_pos.header = msg.header
            self.output.current_pos.pose.position.x = msg.pose.pose.position.x
            self.output.current_pos.pose.position.y = msg.pose.pose.position.y
            self.output.current_pos.pose.orientation.x = msg.pose.pose.orientation.x
            self.output.current_pos.pose.orientation.y = msg.pose.pose.orientation.y
            self.output.current_pos.pose.orientation.z = msg.pose.pose.orientation.z
            self.output.current_pos.pose.orientation.w = msg.pose.pose.orientation.w

    def timer_callback(self):
        if self.new_params:
            self.set_constants()
        self.cmd_pub.publish(self.output)

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------

    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
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


def main(args=None):
    rclpy.init(args=args)
    node = JoyCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
