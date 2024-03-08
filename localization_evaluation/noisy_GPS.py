#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import matplotlib.pyplot as plt
import argparse

class noisy_GPS(Node):
    """
    Create a subscriber node
    """
    def __init__(self, args):
    
        # Initiate the Node class's constructor and give it a name
        super().__init__('noisy_GPS')

        #publishers
        self.noisy_gps_pose_pub = self.create_publisher(PoseStamped, '/filtered_publisher/pose', 10)

        #subscribers
        self.gps_pose_sub = self.create_subscription(
        PoseWithCovarianceStamped,
        '/gps_bot/pose',
        self.gps_pose_callback,
        qos_profile_sensor_data)

    

    def gps_pose_callback(self, msg):
        """
        Callback function.
        """
        noisy_gps = PoseStamped()
        noisy_gps.header = msg.header
        noisy_gps.pose = msg.pose.pose
        noisy_gps.pose.position.x = msg.pose.pose.position.x + np.random.normal(0, 1)
        noisy_gps.pose.position.y = msg.pose.pose.position.y + np.random.normal(0, 1)
        noisy_gps.pose.position.z = msg.pose.pose.position.z + np.random.normal(0, 1)
        self.noisy_gps_pose_pub.publish(noisy_gps)
    
def main(args=None):
    rclpy.init(args=args)
    gps = noisy_GPS(args)
    rclpy.spin(gps)
    rclpy.shutdown()