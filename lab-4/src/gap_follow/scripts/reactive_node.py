#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class ReactiveFollowGap(Node):
    """
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """

    def __init__(self):
        super().__init__('reactive_node')

        # Params
        self.declare_parameter('window_size', 5)   # size of window for mean filtering
        self.declare_parameter('forsee_distance', 5.0)   # ranges beyond this threshold are clipped
        self.declare_parameter('forsee_angle', 45.0)   # only consider within -x~x degrees
        self.declare_parameter('disparity_threshold', 0.15)   # min range changes to define a disparity
        self.declare_parameter('bubble_radius', 0.0)   # bubble_radius
        self.declare_parameter('gap_threshold', 2.4)   # threshold to determine if a range is within a gap or not
        self.declare_parameter('following_orientation', 1/2)   # 1/2 means the center, small means right, bigger means left

        self.declare_parameter('kp', 0.4)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter("max_control", 0.8)   # rad

        self.declare_parameter("low_vel", 3.0)
        self.declare_parameter("high_vel", 8.0)
        self.declare_parameter("speed_coefficient", 3.0)

        self.declare_parameter("steer_alpha", 0.2)
        

        # PID Control Params
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_steer = 0.0

        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.lidar_sub_ = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def preprocess_lidar(self, ranges, range_min=0.0, range_max=30.0):
        """ 
        Preprocess the LiDAR scan array. Expert implementation includes:
        1.Setting each value to the mean over some window
        2.Rejecting high values (e.g. > 3m)
        """
        # remove invalid readings
        ranges = np.clip(ranges, range_min, range_max)

        # clip out high values (tweak 4)
        forsee_distance = self.get_parameter('forsee_distance').get_parameter_value().double_value
        ranges = np.clip(ranges, 0, forsee_distance)

        # average over window
        window_size = self.get_parameter('window_size').get_parameter_value().integer_value
        window = np.ones(window_size) / window_size
        ranges = ndimage.convolve1d(ranges, window, mode='nearest')

        return ranges

    def find_disparities(self, ranges):
        """ 
        Find disparity indices in the LiDAR readings
        """
        d_t = self.get_parameter('disparity_threshold').get_parameter_value().double_value
        disparities = np.abs(np.diff(ranges, prepend=0))
        indices = np.where(disparities > d_t)[0]
        
        return disparities, indices
    
    def zerolize_inside_bubble(self, front_ranges, bubble_indices, bubble_radius):
        """ 
        set beam ranges inside bubbles to 0
        """
        zero_mask = np.zeros_like(front_ranges, dtype=bool)
        for bubble_idx in bubble_indices:
            if bubble_radius < front_ranges[bubble_idx]:
                theta = abs(np.arcsin(bubble_radius / front_ranges[bubble_idx]))
            else:
                theta = np.pi / 2
            radius_to_idx_range = int(theta / self.angle_increment)
            min_idx = max(bubble_idx - radius_to_idx_range, 0)
            max_idx = min(bubble_idx + radius_to_idx_range, len(front_ranges) - 1)
            zero_mask[min_idx:max_idx] = True
        front_ranges[zero_mask] = 0.0
        return front_ranges

    
    def find_widest_gap(self, nums):
        """ 
        find the widest gap
        """
        gap_threshold = self.get_parameter('gap_threshold').get_parameter_value().double_value
        max_start = None
        max_end = None
        max_width = 0
        current_start = None
        current_count = 0
        
        for i, num in enumerate(nums):
            if num > gap_threshold:
                if current_count == 0:
                    # first free point
                    current_start = i
                current_count += 1
            else:
                if current_count >= 5:
                    # found a valid gap
                    if current_count > max_width:
                        # current widest gap
                        max_start = current_start
                        max_end = i - 1
                        max_width = current_count
                current_count = 0
        
        # situation that the end of the list is within the gap
        if current_count >= 5 and current_count > max_width:
            max_start = current_start
            max_end = len(nums) - 1
            max_width = current_count
        
        if max_width == 0:
            return 0, len(nums), len(nums)
        else:
            return max_start, max_end, max_width

    def find_best_point(self, start_i, end_i, ranges, angles):
        """ 
        Start_i & end_i are start and end indices of max-gap range, respectively
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        If more than one furthest points exist, choose the one with the least steering angle
        """
        following_orientation = self.get_parameter('following_orientation').get_parameter_value().double_value
        gap_ranges = ranges[start_i:end_i + 1]
        gap_angles = angles[start_i:end_i + 1]
        self.max_dist = np.max(gap_ranges)
        print(self.max_dist)
        best_angle_center = round((end_i - start_i) * following_orientation + start_i)
        return best_angle_center

    def get_steer(self, error):
        """ Get desired steering angle by PID
        """
        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value
        max_control = self.get_parameter('max_control').get_parameter_value().double_value
        alpha = self.get_parameter('steer_alpha').get_parameter_value().double_value

        d_error = error - self.prev_error
        self.prev_error = error
        self.integral += error
        steer = kp * error + ki * self.integral + kd * d_error
        new_steer = np.clip(steer, -max_control, max_control)
        new_steer = alpha * new_steer + (1 - alpha) * self.prev_steer
        self.prev_steer = new_steer

        return new_steer

    def get_velocity(self, error):
        """ Get desired velocity based on current error
        """
        # Speed is exponential w.r.t error
        low_vel = self.get_parameter('low_vel').get_parameter_value().double_value
        high_vel = self.get_parameter('high_vel').get_parameter_value().double_value
        sc = self.get_parameter('speed_coefficient').get_parameter_value().double_value

        return (high_vel - low_vel) * np.exp(-abs(error) * sc) + low_vel

    def lidar_callback(self, data: LaserScan):
        """ 
        Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        
        # 1. read lidar data
        N = len(data.ranges)
        ranges = np.array(data.ranges)
        angles = np.arange(N) * data.angle_increment + data.angle_min
        self.angle_increment = data.angle_increment

        # 2. preprocess lidar data
        ranges = self.preprocess_lidar(ranges, data.range_min, data.range_max)

        # 3. foresee within some angle range
        forsee_angle = self.get_parameter('forsee_angle').get_parameter_value().double_value
        front_ranges = ranges[np.where((angles > - forsee_angle / 180 * np.pi) & (angles < forsee_angle / 180 * np.pi))[0]]
        front_angles = angles[np.where((angles > - forsee_angle / 180 * np.pi) & (angles < forsee_angle / 180 * np.pi))[0]]

        # 4. find disparities
        _, bubble_indices = self.find_disparities(front_ranges)

        # 5. eliminate all points inside 'bubble' (set them to zero)
        bubble_radius = self.get_parameter('bubble_radius').get_parameter_value().double_value
        front_ranges = self.zerolize_inside_bubble(front_ranges, bubble_indices, bubble_radius)

        # 6. find widest gap
        gap_start_idx, gap_end_idx, width = self.find_widest_gap(front_ranges)
        # self.get_logger().info("Gap: (%d, %d, %d)" % (gap_start_idx, gap_end_idx, width))

        # 7. find the best point in the gap
        best_idx = self.find_best_point(gap_start_idx, gap_end_idx, front_ranges, front_angles)

        # Get speed and steering angle
        steering_error = front_angles[best_idx]
        speed = self.get_velocity(steering_error)
        steer = self.get_steer(steering_error)
        if self.max_dist < 4.0:
            steer = steer / 2

        # Publish Drive message
        # self.get_logger().info("Error: %0.2f,\t Steer: %0.2f,\t Vel: %0.2f" % (np.rad2deg(steering_error),
        #                                                                        np.rad2deg(steer),
        #                                                                        speed))
        message = AckermannDriveStamped()
        message.drive.speed = speed
        message.drive.steering_angle = steer
        self.drive_pub_.publish(message)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
