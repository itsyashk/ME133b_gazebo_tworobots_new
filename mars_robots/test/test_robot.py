#!/usr/bin/env python3

import unittest
import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import time

class TestRobot(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_robot')
        cls.robot_name = 'robot1'  # Can be parameterized for different robots

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.cmd_vel_received = False
        self.odom_received = False
        self.scan_received = False
        self.tf_received = False

        # Create subscribers
        self.cmd_vel_sub = self.node.create_subscription(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.odom_sub = self.node.create_subscription(
            Odometry,
            f'/{self.robot_name}/odom',
            self.odom_callback,
            10)
        
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.scan_callback,
            10)
        
        self.tf_sub = self.node.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)

        # Create publisher
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            f'/{self.robot_name}/cmd_vel',
            10)

    def cmd_vel_callback(self, msg):
        self.cmd_vel_received = True

    def odom_callback(self, msg):
        self.odom_received = True

    def scan_callback(self, msg):
        self.scan_received = True

    def tf_callback(self, msg):
        self.tf_received = True

    def test_topics_active(self):
        """Test if all required topics are active and receiving messages."""
        timeout = 5.0  # seconds
        start_time = time.time()

        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if all([self.odom_received, self.scan_received, self.tf_received]):
                break

        self.assertTrue(self.odom_received, "Odometry topic not receiving messages")
        self.assertTrue(self.scan_received, "LaserScan topic not receiving messages")
        self.assertTrue(self.tf_received, "TF topic not receiving messages")

    def test_cmd_vel_publishing(self):
        """Test if the robot responds to velocity commands."""
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = 0.1

        # Publish command
        self.cmd_vel_pub.publish(msg)

        # Wait for response
        timeout = 2.0  # seconds
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.cmd_vel_received:
                break

        self.assertTrue(self.cmd_vel_received, "Robot not responding to velocity commands")

if __name__ == '__main__':
    pytest.main([__file__])