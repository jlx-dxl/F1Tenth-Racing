#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""
import numpy as np
import math
import random
from time import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Int16

from RRT import RRTStar, TreeNode

class DynamicPlanner(Node):
    """
    Class for planner node
    """

    def __init__(self):
        super().__init__('dynamic_planner_node')

        # ROS Params
        self.declare_parameter('visualize',True)
        self.declare_parameter('real_test',False)

        self.declare_parameter('lookahead_distance', 4.0)
        self.declare_parameter('lookahead_attenuation', 0.6)
        self.declare_parameter('lookahead_idx', 20)
        self.declare_parameter('lookbehind_idx', 0)

        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter("max_control", 0.36)
        self.declare_parameter("steer_alpha", 1.0)

        self.declare_parameter('grid_xmin', 0.0)
        self.declare_parameter('grid_xmax', 5.0)
        self.declare_parameter('grid_ymin', -1.5)
        self.declare_parameter('grid_ymax', 1.5)
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('plot_resolution', 0.25)
        self.declare_parameter('grid_safe_dist', 0.2)
        self.declare_parameter('goal_safe_dist', 0.5)

        self.declare_parameter('use_rrt', True)
        self.declare_parameter('collision_tol', 0.2)
        self.declare_parameter('expand_dis', 0.5)
        self.declare_parameter('path_resolution', 0.1)
        self.declare_parameter('goal_sample_rate', 6.0)
        self.declare_parameter('max_iter', 100)
        self.declare_parameter('circle_dist', 0.3)
        self.declare_parameter('early_stop', True)
        self.declare_parameter('smooth', True)
        self.declare_parameter('smoother_type', "dp")
        self.declare_parameter('smoother_iter', 100)
        
        self.declare_parameter('opp_racecar_speed', 3.0)

        # PID Control Params
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_steer = 0.0

        # Global Map Params
        self.real_test = self.get_parameter('real_test').get_parameter_value().bool_value
        csv_loc = '/home/lucien/ESE6150/traj_map.csv'

        waypoints = np.loadtxt(csv_loc, delimiter=',',skiprows=1)
        self.num_pts = len(waypoints)
        self.waypoint_x = waypoints[:, 0]
        self.waypoint_y = waypoints[:, 1]
        self.waypoint_v = waypoints[:, 2]
        self.waypoint_yaw = waypoints[:, 3]
        self.waypoint_pos = waypoints[:, 0:2]
        self.v_max = np.max(self.waypoint_v)
        self.v_min = np.min(self.waypoint_v)

        # Local Map Params
        self.grid = None
        self.rrt_tree = None
        self.rrt_path = None
        self.smooth_path = None

        # Car State Params
        self.curr_global_pos, self.curr_global_yaw = None, None
        self.goal_local_pos = None
        self.goal_global_pos = None

        # Other Params
        self.frame_cnt = 0

        # Topics & Subs, Pubs
        pose_topic = "/pf/viz/inferred_pose" if self.real_test else "/ego_racecar/odom"
        scan_topic = "/scan"
        drive_topic = '/drive'
        
        opp_pose_topic = "/opp_racecar/odom"
        opp_drive_topic = '/opp_drive'

        grid_topic = '/grid'
        rrt_topic = '/rrt'
        smooth_topic = '/smooth'
        waypoint_topic = '/waypoint'
        path_topic = '/global_path'
        fps_topic = '/fps'

        self.timer = self.create_timer(1.0, self.timer_callback)

        if self.real_test:
            self.pose_sub_ = self.create_subscription(PoseStamped, pose_topic, self.pose_callback, 1)
            self.opp_pose_sub_ = self.create_subscription(PoseStamped, opp_pose_topic, self.opp_pose_callback, 1)
        else:
            self.pose_sub_ = self.create_subscription(Odometry, pose_topic, self.pose_callback, 1)
            self.opp_pose_sub_ = self.create_subscription(Odometry, opp_pose_topic, self.opp_pose_callback, 1)

        self.scan_sub_ = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 1)
        self.drive_pub_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.grid_pub_ = self.create_publisher(MarkerArray, grid_topic, 10)
        self.rrt_pub_ = self.create_publisher(Marker, rrt_topic, 10)
        self.smooth_pub_ = self.create_publisher(Marker, smooth_topic, 10)
        self.waypoint_pub_ = self.create_publisher(Marker, waypoint_topic, 10)
        self.path_pub_ = self.create_publisher(Marker, path_topic, 10)
        self.fps_pub_ = self.create_publisher(Int16, fps_topic, 10)
        self.opp_drive_pub_ = self.create_publisher(AckermannDriveStamped, opp_drive_topic, 10)


############################################ Callback Functions #####################################


    def timer_callback(self):
        fps = Int16()
        fps.data = self.frame_cnt
        self.frame_cnt = 0
        self.fps_pub_.publish(fps)
        # self.get_logger().info('fps: %d' % fps.data)


    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args:
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        ranges = np.array(scan_msg.ranges)
        ranges = np.clip(ranges, scan_msg.range_min, scan_msg.range_max)

        xmin = self.get_parameter('grid_xmin').get_parameter_value().double_value
        xmax = self.get_parameter('grid_xmax').get_parameter_value().double_value
        ymin = self.get_parameter('grid_ymin').get_parameter_value().double_value
        ymax = self.get_parameter('grid_ymax').get_parameter_value().double_value
        resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        grid_safe_dist = self.get_parameter('grid_safe_dist').get_parameter_value().double_value

        nx = int((xmax - xmin) / resolution) + 1
        ny = int((ymax - ymin) / resolution) + 1

        x = np.linspace(xmin, xmax, nx)
        y = np.linspace(ymin, ymax, ny)
        y, x = np.meshgrid(y, x)
        rho = np.sqrt(x ** 2 + y ** 2)
        phi = np.arctan2(y, x)

        ray_idx = ((phi - scan_msg.angle_min) / scan_msg.angle_increment).astype(int)
        obs_rho = ranges[ray_idx]

        self.grid = np.where(np.abs(rho - obs_rho) < grid_safe_dist, 1.0, 0.0)
        self.grid = np.dstack((self.grid, x, y))  # (h, w, 3)


    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args:
            pose_msg (PoseStamped or Odometry): incoming message from subscribed topic
        Returns:

        """
        # Get speed, steer, car position and yaw
        use_rrt = self.get_parameter('use_rrt').get_parameter_value().bool_value

        res = self.get_control(pose_msg, use_rrt=use_rrt)

        if not res:
            return

        speed = res[0]
        steer = res[1]
        self.curr_global_pos = res[2]
        self.curr_global_yaw = res[3]
        self.goal_global_pos = res[4]
        self.goal_local_pos = res[5]

        # Publish drive message
        message = AckermannDriveStamped()
        message.drive.speed = speed
        message.drive.steering_angle = steer
        self.get_logger().info('speed: %f, steer: %f' % (speed, steer))
        self.drive_pub_.publish(message)

        # Visualize waypoint
        visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        if visualize:
            self.visualize_occupancy_grid()
            self.visualize_rrt()
            self.visualize_smooth_path()
            self.visualize_waypoints()

        # Increase frame count
        self.frame_cnt += 1

        return None
    
    def opp_pose_callback(self, pose_msg):
        """
        The opponent pose callback

        Args:
            pose_msg (PoseStamped or Odometry): incoming message from subscribed topic
        Returns:

        """
        # Get speed, steer, car position and yaw
        res = self.get_control(pose_msg, use_rrt=False)

        if not res:
            return

        speed = self.get_parameter('opp_racecar_speed').get_parameter_value().double_value
        steer = res[1]

        # Publish drive message
        message = AckermannDriveStamped()
        message.drive.speed = speed
        message.drive.steering_angle = steer
        self.opp_drive_pub_.publish(message)

############################################ Helper Functions #####################################

    def get_control(self, pose_msg, use_rrt=False):
        """
        This method should calculate the desired speed and steering angle and other status

        Args:
            pose_msg (PoseStamped or Odometry): incoming message from subscribed topic
            use_rrt (bool): whether to use RRT local planner
        Returns:
            speed (float): car target speed
            steer (float): car target steering angle
            curr_global_pos (numpy.ndarray): car current position in map frame
            curr_global_yaw (float): car current yaw angle in map frame
            goal_global_pos (numpy.ndarray): car target position in map frame
            goal_local_pos (numpy.ndarray): car target position in car frame

        """
        # Read pose data
        if self.real_test:
            curr_x = pose_msg.pose.position.x
            curr_y = pose_msg.pose.position.y
            curr_quat = pose_msg.pose.orientation
        else:
            curr_x = pose_msg.pose.pose.position.x
            curr_y = pose_msg.pose.pose.position.y
            curr_quat = pose_msg.pose.pose.orientation

        curr_global_pos = np.array([curr_x, curr_y])
        curr_global_yaw = math.atan2(2 * (curr_quat.w * curr_quat.z + curr_quat.x * curr_quat.y),
                                     1 - 2 * (curr_quat.y ** 2 + curr_quat.z ** 2))

        # Wait until laser scan available
        if self.grid is None:
            return

        # Find index of the current point
        distances = np.linalg.norm(self.waypoint_pos - curr_global_pos, axis=1)
        curr_idx = np.argmin(distances)

        # Get lookahead distance
        L = self.get_lookahead_dist(curr_idx)

        # Search safe global target waypoint
        goal_safe_dist = self.get_parameter('goal_safe_dist').get_parameter_value().double_value
        while True:
            # Binary search goal waypoint to track
            goal_idx = curr_idx
            while distances[goal_idx] <= L:
                goal_idx = (goal_idx + 1) % self.num_pts

            left = self.waypoint_pos[(goal_idx - 1) % self.num_pts, :]
            right = self.waypoint_pos[goal_idx % self.num_pts, :]

            while True:
                mid = (left + right) / 2
                dist = np.linalg.norm(mid - curr_global_pos)
                if abs(dist - L) < 1e-2:
                    goal_global_pos = mid
                    break
                elif dist > L:
                    right = mid
                else:
                    left = mid

            # Transform goal point to vehicle frame of reference
            R = np.array([[np.cos(curr_global_yaw), np.sin(curr_global_yaw)],
                          [-np.sin(curr_global_yaw), np.cos(curr_global_yaw)]])
            goal_local_pos = R @ np.array([goal_global_pos[0] - curr_global_pos[0],
                                           goal_global_pos[1] - curr_global_pos[1]])

            # Check if target point collision free
            if self.dist_to_grid(goal_local_pos) > goal_safe_dist:
                break
            L *= 1.1

        # Use RRT for local planning
        if use_rrt:
            self.rrt_tree, self.rrt_path, self.smooth_path = self.local_planning(goal_local_pos)

            if not self.rrt_path or len(self.rrt_path) == 2:
                y_error = goal_local_pos[1]
            else:
                y_error = self.smooth_path[1][1] if self.smooth_path else self.rrt_path[1][1]

        else:
            y_error = goal_local_pos[1]

        # Get desired speed and steering angle
        speed = self.waypoint_v[curr_idx % self.num_pts]
        gamma = 2 / L ** 2
        error = gamma * y_error
        steer = self.get_steer(error)

        return speed, steer, curr_global_pos, curr_global_yaw, goal_global_pos, goal_local_pos


    def get_lookahead_dist(self, curr_idx):
        """
        This method should calculate the lookahead distance based on past and future waypoints

        Args:
            curr_idx (ndarray[int]): closest waypoint index
        Returns:
            lookahead_dist (float): lookahead distance

        """
        L = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        lookahead_idx = self.get_parameter('lookahead_idx').get_parameter_value().integer_value
        lookbehind_idx = self.get_parameter('lookbehind_idx').get_parameter_value().integer_value
        slope = self.get_parameter('lookahead_attenuation').get_parameter_value().double_value

        yaw_before = self.waypoint_yaw[(curr_idx - lookbehind_idx) % self.num_pts]
        yaw_after = self.waypoint_yaw[(curr_idx + lookahead_idx) % self.num_pts]
        yaw_diff = abs(yaw_after - yaw_before)
        if yaw_diff > np.pi:
            yaw_diff = yaw_diff - 2 * np.pi
        if yaw_diff < -np.pi:
            yaw_diff = yaw_diff + 2 * np.pi
        yaw_diff = abs(yaw_diff)
        if yaw_diff > np.pi / 2:
            yaw_diff = np.pi / 2
        L = max(0.5, L * (np.pi / 2 - yaw_diff * slope) / (np.pi / 2))

        return L


    def dist_to_grid(self, pos):
        """
        Calculate distance to occupancy grid

        Args:
            pos (numpy.ndarray or (x, y)): current position
        Returns:
            dist (float): distance to occupancy grid

        """
        grid_v = self.grid[:, :, 0].flatten()
        grid_x = self.grid[:, :, 1].flatten()
        grid_y = self.grid[:, :, 2].flatten()

        grid_x = grid_x[grid_v == 1.0]
        grid_y = grid_y[grid_v == 1.0]
        grid_pos = np.vstack((grid_x, grid_y))

        dist = np.min(np.linalg.norm(grid_pos.T - pos, axis=-1))

        return dist


    def get_steer(self, error):
        """
        Get desired steering angle by PID

        Args:
            error (float): current error
        Returns:
            new_steer (float): desired steering angle

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


    def local_planning(self, goal_pos):
        """
        real time local planning using RRTstar

        Args:
            goal_pos (_type_): _description_

        Returns:
            rrt.tree: (list): 
            path(list):
            smooth_path(list):
        """
        collision_tol = self.get_parameter('collision_tol').get_parameter_value().double_value
        expand_dis = self.get_parameter('expand_dis').get_parameter_value().double_value
        path_resolution = self.get_parameter('path_resolution').get_parameter_value().double_value
        goal_sample_rate = self.get_parameter('goal_sample_rate').get_parameter_value().double_value
        max_iter = self.get_parameter('max_iter').get_parameter_value().integer_value
        circle_dist = self.get_parameter('circle_dist').get_parameter_value().integer_value
        early_stop = self.get_parameter('early_stop').get_parameter_value().bool_value
        smooth = self.get_parameter('smooth').get_parameter_value().bool_value
        smoother_type = self.get_parameter('smoother_type').get_parameter_value().string_value
        smoother_iter = self.get_parameter('smoother_iter').get_parameter_value().integer_value

        rrt = RRTStar(start=(0, 0),
                  goal=goal_pos,
                  occupancy_grid_map=self.grid,
                  collision_tolerance=collision_tol,
                  expand_dis_threshold=expand_dis,
                  path_resolution=path_resolution,
                  goal_sample_rate=goal_sample_rate,
                  max_iter=max_iter,
                  neighborhood_range=circle_dist,
                  if_early_stop=early_stop,
                  smooth=smooth,
                  smoother_type=smoother_type,
                  smoother_iter=smoother_iter)
        path, smooth_path = rrt.planning()

        return rrt.tree, path, smooth_path


########################################## Visualization ##################################################

    def visualize_occupancy_grid(self):
        if self.grid is None:
            return

        grid_resolution = self.get_parameter('grid_resolution').get_parameter_value().double_value
        plot_resolution = self.get_parameter('plot_resolution').get_parameter_value().double_value
        down_sample = max(1, int(plot_resolution / grid_resolution))

        grid = self.grid.copy()
        grid = grid[::down_sample, ::down_sample, :]  # down sample for faster plotting

        grid_v = grid[:, :, 0].flatten()
        grid_x = grid[:, :, 1].flatten()
        grid_y = grid[:, :, 2].flatten()

        # Transform occupancy grid into map frame
        pos = np.vstack((grid_x.flatten(), grid_y.flatten()))
        R = np.array([[np.cos(self.curr_global_yaw), -np.sin(self.curr_global_yaw)],
                      [np.sin(self.curr_global_yaw), np.cos(self.curr_global_yaw)]])
        grid_x, grid_y = R @ pos + self.curr_global_pos.reshape(-1, 1)

        # Publish occupancy grid
        marker_arr = MarkerArray()

        for i in range(len(grid_v)):
            if grid_v[i] == 0:
                continue

            marker = Marker()
            marker.header.frame_id = '/map'
            marker.id = i
            marker.ns = 'occupancy_grid_%u' % i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = grid_x[i]
            marker.pose.position.y = grid_y[i]

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            marker.lifetime.nanosec = int(1e8)

            marker_arr.markers.append(marker)

        self.grid_pub_.publish(marker_arr)


    def visualize_rrt(self):
        if not self.rrt_tree:
            return

        # Rotation matrix from car frame to map frame
        R = np.array([[np.cos(self.curr_global_yaw), -np.sin(self.curr_global_yaw)],
                      [np.sin(self.curr_global_yaw), np.cos(self.curr_global_yaw)]])

        # Publish rrt tree and path
        line_list = Marker()
        line_list.header.frame_id = '/map'
        line_list.id = 0
        line_list.ns = 'rrt'
        line_list.type = Marker.LINE_LIST
        line_list.action = Marker.ADD

        line_list.scale.x = 0.1
        line_list.scale.y = 0.1
        line_list.scale.z = 0.1

        line_list.points = []

        for node in self.rrt_tree:
            if node.parent is None:
                continue

            if not self.rrt_path:
                color = (0.0, 0.0, 1.0)
            else:
                dist = np.linalg.norm(np.array(self.rrt_path) - np.array([node.x, node.y]), axis=-1)
                on_path = np.any(dist == 0.0)
                color = (0.0, 1.0, 0.0) if on_path else (0.0, 0.0, 1.0)

            # Add first point
            this_point = Point()

            local_pos = np.array([node.parent.x, node.parent.y], dtype=float)
            global_pos = R @ local_pos + self.curr_global_pos
            this_point.x = global_pos[0]
            this_point.y = global_pos[1]
            line_list.points.append(this_point)

            this_color = ColorRGBA()
            this_color.r = color[0]
            this_color.g = color[1]
            this_color.b = color[2]
            this_color.a = 1.0
            line_list.colors.append(this_color)

            # Add second point
            this_point = Point()

            local_pos = np.array([node.x, node.y], dtype=float)
            global_pos = R @ local_pos + self.curr_global_pos
            this_point.x = global_pos[0]
            this_point.y = global_pos[1]
            line_list.points.append(this_point)

            this_color = ColorRGBA()
            this_color.r = color[0]
            this_color.g = color[1]
            this_color.b = color[2]
            this_color.a = 1.0
            line_list.colors.append(this_color)

        self.rrt_pub_.publish(line_list)


    def visualize_smooth_path(self):
        if not self.smooth_path:
            return

        # Rotation matrix from car frame to map frame
        R = np.array([[np.cos(self.curr_global_yaw), -np.sin(self.curr_global_yaw)],
                      [np.sin(self.curr_global_yaw), np.cos(self.curr_global_yaw)]])

        # Publish rrt tree and path
        line_list = Marker()
        line_list.header.frame_id = '/map'
        line_list.id = 0
        line_list.ns = 'smooth_path'
        line_list.type = Marker.LINE_LIST
        line_list.action = Marker.ADD

        line_list.scale.x = 0.1
        line_list.scale.y = 0.1
        line_list.scale.z = 0.1

        line_list.points = []

        color = (255.0, 105.0, 180.0)

        for idx in range(len(self.smooth_path) - 1):
            # Add first point
            this_point = Point()

            local_pos = np.array(self.smooth_path[idx], dtype=float)
            global_pos = R @ local_pos + self.curr_global_pos
            this_point.x = global_pos[0]
            this_point.y = global_pos[1]
            line_list.points.append(this_point)

            this_color = ColorRGBA()
            this_color.r = color[0] / 255.0
            this_color.g = color[1] / 255.0
            this_color.b = color[2] / 255.0
            this_color.a = 1.0
            line_list.colors.append(this_color)

            # Add second point
            this_point = Point()

            local_pos = np.array(self.smooth_path[idx + 1], dtype=float)
            global_pos = R @ local_pos + self.curr_global_pos
            this_point.x = global_pos[0]
            this_point.y = global_pos[1]
            line_list.points.append(this_point)

            this_color = ColorRGBA()
            this_color.r = color[0] / 255.0
            this_color.g = color[1] / 255.0
            this_color.b = color[2] / 255.0
            this_color.a = 1.0
            line_list.colors.append(this_color)

        self.smooth_pub_.publish(line_list)


    def visualize_waypoints(self):
        # Publish all waypoints
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.id = 0
        marker.ns = 'global_planner'
        marker.type = 4
        marker.action = 0
        marker.points = []
        marker.colors = []
        for i in range(self.num_pts + 1):
            this_point = Point()
            this_point.x = self.waypoint_x[i % self.num_pts]
            this_point.y = self.waypoint_y[i % self.num_pts]
            marker.points.append(this_point)

            this_color = ColorRGBA()
            speed_ratio = (self.waypoint_v[i % self.num_pts] - self.v_min) / (self.v_max - self.v_min)
            this_color.a = 1.0
            this_color.r = (1 - speed_ratio)
            this_color.g = speed_ratio
            marker.colors.append(this_color)

        this_scale = 0.1
        marker.scale.x = this_scale
        marker.scale.y = this_scale
        marker.scale.z = this_scale

        marker.pose.orientation.w = 1.0

        self.path_pub_.publish(marker)

        # Publish target waypoint
        marker = Marker()
        marker.header.frame_id = '/map'
        marker.id = 0
        marker.ns = 'target_waypoint'
        marker.type = 1
        marker.action = 0
        marker.pose.position.x = self.goal_global_pos[0]
        marker.pose.position.y = self.goal_global_pos[1]

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        this_scale = 0.2
        marker.scale.x = this_scale
        marker.scale.y = this_scale
        marker.scale.z = this_scale

        marker.pose.orientation.w = 1.0

        self.waypoint_pub_.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    print("Motion Planner Initialized")
    planner_node = DynamicPlanner()
    rclpy.spin(planner_node)

    planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
