#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, TwistStamped, WrenchStamped
from sensor_msgs.msg import Joy, Imu
from nav_msgs.msg import Odometry
from bur_msgs.msg import Command, ThrusterCommand

import tf2_ros
from tf2_ros import TransformException
import math

from rcl_interfaces.msg import SetParametersResult
import numpy as np
import os

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
        self.declare_parameter('update_rate', 20)

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
            WrenchStamped,
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
        self.controller_msg = WrenchStamped()
        self.thruster_msg =  ThrusterCommand()
        
        self.imu_msg = Imu()
        self.imu_euler_msg = Vector3Stamped()

        self.imu_initialized = False
        self.counter = 0


    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------

    def command_callback(self, msg: Command):
        self.command_msg = msg

    def controller_callback(self, msg: WrenchStamped):
        self.controller_msg = msg

    def thruster_callback(self, msg: ThrusterCommand):
        self.thruster_msg = msg


    def imu_callback(self, msg: Imu):
        self.imu_msg = msg
        self.imu_initialized = True

        # Convert quaternion → Euler and publish
        # Note: tf2 quaternion convention is (x, y, z, w)
        roll, pitch, yaw = quat_to_rpy(
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
        # This format is heavily inspired by https://github.com/vectr-ucla/direct_lidar_inertial_odometry

        if not self.imu_initialized:
            self.counter += 1
            if self.counter == 100:
                print("Waiting for IMU...")
                self.counter = 0
            return

        # Magic numbers
        column_width = 9
        interior_width = 21 + 6 * (9 + 1) # = 81

        def unpack_vector(vec):
            return [vec.x, vec.y, vec.z]
        def format_vector(vec):
            return [f'{float(num):>9.4f}' for num in unpack_vector(vec)]
        def quat_msg_to_rpy(msg):
            return quat_to_rpy(msg.x, msg.y, msg.z, msg.w)

        print("\033[2J\033[1;1H")

        print('+' + '-' * (21 + 6 * (9 + 1)) + '+')
        print('|' + '{:^81}'.format('Bruin Underwater Robotics') + '|')

        print('+' + '-' * (21 + 6 * (9 + 1)) + '+')
        print('| ' + ' '*20 + ' '.join(f'{s:^9}' for s in ['linearX', 'linearY', 'linearZ', 'roll', 'pitch', 'yaw']) + ' |')
        print('| ' + '{:<20}'.format('Current position:') + ' '.join(format_vector(self.command_msg.current_pos.pose.position)+[f'{float(num):>9.4f}' for num in quat_msg_to_rpy(self.command_msg.current_pos.pose.orientation)])  + ' |')
        print('| ' + '{:<20}'.format('Target position:') + ' '.join(format_vector(self.command_msg.target_pos.pose.position)+[f'{float(num):>9.4f}' for num in quat_msg_to_rpy(self.command_msg.target_pos.pose.orientation)]) + ' |')

        print('|' + ' ' * (21 + 6 * (9 + 1)) + '|')
        print('| ' + '{:<20}'.format('Current velocity:') + ' '.join(format_vector(self.command_msg.current_vel.twist.linear)+format_vector(self.command_msg.current_vel.twist.angular)) + ' |')
        print('| ' + '{:<20}'.format('Target velocity:') + ' '.join(format_vector(self.command_msg.target_vel.twist.linear)+format_vector(self.command_msg.target_vel.twist.angular)) + ' |')

        print('|' + ' ' * (21 + 6 * (9 + 1)) + '|')
        print('| ' + '{:<20}'.format('Control effort:') + ' '.join(format_vector(self.controller_msg.wrench.force)+format_vector(self.controller_msg.wrench.torque)) + ' |')
        print('|' + '=' * (21 + 6 * (9 + 1)) + '|')

        if not self.thruster_msg.thrusters or len(self.thruster_msg.thrusters) < 8:
            if not self.thruster_msg.thrusters:
                self.thruster_msg.thrusters = []
            self.thruster_msg.thrusters = list(self.thruster_msg.thrusters) + [0.0] * (8 - len(self.thruster_msg._thrusters))

        thruster_dirs = [
            '+x, -y',
            '-z',
            '-x, -y',
            '+z',
            '-x, -y',
            '+z',
            '-x, +y',
            '-z'
        ]

        thruster_text = ['Thrusters:']
        thruster_text += [' '*3+f'{str(i)}: {float(num):>9.4f} ({thruster_dirs[i]})' for i, num in enumerate(self.thruster_msg.thrusters)]
        bot_ascii = [
            '',
            '(0)' + ' '*1+'+'+'-'*11+'+' + ' (2)',
            '|'+' '*11+'|',
            '(1)'+' '*11+'(3)',
            '|'+' '*5 + '^' + ' ' * 5 + '|',
            '|'+' '*5 + '|' + ' ' * 5 + '|',
            '(5)'+' '*11+'(7)',
            '|'+' '*11+'|',
            '(4)' + ' '*1+'+'+'-'*11+'+' + ' (6)',
            # '+'+'-'*10+'+',
            # '(4)' + ' '*14 + '(6)',
        ]

        for i in range(len(thruster_text)):
            print('| ' + '{:<40}'.format(thruster_text[i]) + '{:^40}'.format(bot_ascii[i]) + '|')

        print('|' + ' ' * (21 + 6 * (9 + 1)) + '|')
        print('| Buttons: ' + '{:<71}'.format('  '.join(f'{int(b)}' for b in self.thruster_msg.buttons)) + '|')

        print('+' + '-' *(21 + 6 * (9 + 1)) + '+')


def main(args=None):
    rclpy.init(args=args)
    node = TopicViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
