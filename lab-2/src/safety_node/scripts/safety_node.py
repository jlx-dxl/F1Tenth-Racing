#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.0
        self.declare_parameter('ittc_threshold', 1.0)
        self.ittc_threshold = 0.0
        self.adaptive_ittc_threshold = 0.0
        self.scan_sub_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub_ = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)

    def odom_callback(self, odom_msg: Odometry):
        self.speed = odom_msg.twist.twist.linear.x
        # self.get_logger().info("forward-speed: %f" % self.speed)
        # low speed:
        if self.speed < 0.6:
            self.adaptive_ittc_threshold = 1.2 * self.ittc_threshold
        # medium speed
        elif (self.speed >= 0.6) & (self.speed < 1.2):
            self.adaptive_ittc_threshold = 0.8 * self.ittc_threshold
        # high speed
        else:
            self.adaptive_ittc_threshold = 0.6 * self.ittc_threshold

    def scan_callback(self, scan_msg: LaserScan):
        self.ittc_threshold = self.get_parameter('ittc_threshold').get_parameter_value().double_value
        
        n_ranges = len(scan_msg.ranges)   # 1080
        ranges = np.array(scan_msg.ranges)   # range datas
        cared_ranges = ranges[int(n_ranges*7/18):int(n_ranges*11/18)]   # for AEB, we only care about the front of the car, so I choose a 60 range (-30~30)
        cared_ranges = np.where(((cared_ranges >= scan_msg.range_min) & (cared_ranges <= scan_msg.range_max)), cared_ranges, np.inf)
        
        angles = np.arange(n_ranges) * scan_msg.angle_increment + scan_msg.angle_min
        cared_angles = angles[int(n_ranges*7/18):int(n_ranges*11/18)]   # angle values corresponding to cared rages 
        
        vels = self.speed * np.cos(cared_angles)
        vels = np.where(vels > 1e-8, vels, 1e-8)
        
        ittc = min(cared_ranges / vels)
        self.get_logger().info("ITTC:%f; Threshold:%f" % (ittc,self.adaptive_ittc_threshold))
        if ittc < self.adaptive_ittc_threshold:
            message = AckermannDriveStamped()
            message.drive.speed = 0.0
            self.publisher_.publish(message)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()