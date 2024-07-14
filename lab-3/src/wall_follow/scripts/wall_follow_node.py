#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        self.declare_parameter('which_wall', "left")   # choose which wall to follow ("left" or "right")
        self.declare_parameter('distance', 0.5)   # distance to the wall
        self.declare_parameter('theta', 70.0)   # paramter to culculat the distance toward the wall
        self.declare_parameter('sample_num',80)   # number of sampling when culculating the distance toward the wall (how many samples between 0 and theta)
        self.declare_parameter('percentile',25)   # eliminate $percentile numbers from up and bottom of a data list
        self.declare_parameter('iqr_threshold',1.5)   # threshold for iqr
        self.declare_parameter('forward_distance', 3.0)   # L in the slides
        self.declare_parameter('steer_angle_threshold', 45)   # steering angle constrain
        self.declare_parameter('plot_switch', "on")   # debug tool, turn it on to see curves ("on" or "off")
        self.declare_parameter('forward_distance_threshold', 10.0)   # threshold of forward distance to help determine if to steer
        self.declare_parameter('error_threshold', 2.0)   # thereshold of error to help determine if to steer
        self.declare_parameter('angle_coefficient', 0.25)   # coeffcient to help tuning steering angle under different situations
        self.plot_switch = self.get_parameter('plot_switch').get_parameter_value().string_value

        if self.plot_switch == "off":
            # PID parameters (without plt)
            self.declare_parameter('kp', 0.005)
            self.declare_parameter('ki', 0.0)
            self.declare_parameter('kd', 0.0000005)
        else:
            # PID parameters (with plt)
            self.declare_parameter('kp', 0.005)
            self.declare_parameter('ki', 0.0000)
            self.declare_parameter('kd', 0.0000005)
        

        # TODO: create subscribers and publishers
        self.scan_sub_ = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.filtered_error = 0.0

        # TODO: store any necessary values you think you'll need
        self.angle_min = 0.0
        self.angle_max = 0.0
        self.angle_increment = 0.0
        self.range_min = 0.0
        self.range_max = 0.0
        
        if self.plot_switch == "on":
            # Set up the plot
            self.x_data = []
            self.y_data_error = []
            self.y_data_forward_dis = []
            self.y_data_angle = []
            self.y_data_speed = []

            self.fig, (self.ax1, self.ax2, self.ax3, self.ax4) = plt.subplots(4, 1, sharex=True)
            self.line_error, = self.ax1.plot(self.x_data, self.y_data_error, label='Error')
            self.line_forward_dis, = self.ax2.plot(self.x_data, self.y_data_forward_dis, label='Forward Dis')
            self.line_angle, = self.ax3.plot(self.x_data, self.y_data_angle, label='Angle')
            self.line_speed, = self.ax4.plot(self.x_data, self.y_data_speed, label='Speed')
            self.ax1.set_title('Error')
            self.ax1.set_xlabel('Time')
            self.ax1.set_ylabel('Error')
            self.ax2.set_title('Forward Dis')
            self.ax2.set_xlabel('Time')
            self.ax2.set_ylabel('Forward Dis')
            self.ax3.set_title('Angle')
            self.ax3.set_xlabel('Time')
            self.ax3.set_ylabel('Angle')
            self.ax4.set_title('Speed')
            self.ax4.set_xlabel('Time')
            self.ax4.set_ylabel('Speed')
            # Enable the grid for both subplots
            self.ax1.grid(True)
            self.ax2.grid(True)
            self.ax3.grid(True)
            self.ax4.grid(True)
            
            # Enable interactive mode
            plt.ion()
            plt.show()
        

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        #TODO: implement
        assert (angle - self.angle_min > 0)
        angle_idx = int((angle - self.angle_min) / self.angle_increment)
        range = range_data[angle_idx]
        # assert (range > self.range_min) and (range < self.range_max)
        
        return range
    
    def eliminate_outlier(self, list_of_num):
        """
        Eliminate the outliers using IQR

        Args:
            list_of_num: input data list

        Returns:
            result_list: output numpy array
        """
        percentile = self.get_parameter('percentile').get_parameter_value().integer_value
        iqr_thershold = self.get_parameter('iqr_threshold').get_parameter_value().double_value

        lower_percentile = np.percentile(np.array(list_of_num),percentile)
        upper_percentile = np.percentile(np.array(list_of_num),100-percentile)
        iqr = upper_percentile - lower_percentile
        lower_bound = lower_percentile - iqr_thershold * iqr
        upper_bound = upper_percentile + iqr_thershold * iqr
        
        result_list = np.array(list_of_num)[(list_of_num>lower_bound)&(list_of_num<upper_bound)]
        
        return result_list


    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left 
        (going counter clockwise in the Levine loop).
        You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        theta = self.get_parameter('theta').get_parameter_value().double_value
        sample_num = self.get_parameter('sample_num').get_parameter_value().integer_value
        which_wall = self.get_parameter('which_wall').get_parameter_value().string_value

        theta_increment = (theta / sample_num) * np.pi / 180
        alpha_list = []
        
        if which_wall == "left":
            b = self.get_range(range_data, np.pi/2)
        elif which_wall == "right":
            b = self.get_range(range_data, -np.pi/2)
                   
        for i in range(sample_num):
            if which_wall == "left":
                a = self.get_range(range_data, np.pi/2 - theta_increment*i)
                alpha = np.arctan2(b - a * np.cos(theta), a * np.sin(theta))
                alpha_list.append(alpha)
            elif which_wall == "right":
                a = self.get_range(range_data, -np.pi/2 + theta_increment*i)
                alpha = np.arctan2(a * np.cos(theta) - b, a * np.sin(theta))
                alpha_list.append(alpha)

        clean_alpha_list = self.eliminate_outlier(alpha_list)
        final_alpha = np.mean(clean_alpha_list)
        
        L = sample_num = self.get_parameter('forward_distance').get_parameter_value().double_value
        if which_wall == "left":
            error = b * np.cos(final_alpha) - L * np.sin(final_alpha) - dist
        elif which_wall == "right":
            error = dist - (b * np.cos(final_alpha) - L * np.sin(final_alpha))
                
        return error

    def pid_control(self, forward_distance, error):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        
        # TODO: Use kp, ki & kd to implement a PID controller
        kp = self.get_parameter('kp').get_parameter_value().double_value
        ki = self.get_parameter('ki').get_parameter_value().double_value
        kd = self.get_parameter('kd').get_parameter_value().double_value
        fdt = self.get_parameter('forward_distance_threshold').get_parameter_value().double_value
        et = self.get_parameter('error_threshold').get_parameter_value().double_value
        ac = self.get_parameter('angle_coefficient').get_parameter_value().double_value
        # self.get_logger().info("kp:%f; ki:%f; kd:%f" % (kp,ki,kd))
        
        d_error = error - self.prev_error
        self.prev_error = error
        self.integral += error

        angle = kp * error + ki * self.integral + kd * d_error
        angle = angle * 180 / np.pi
        
        if forward_distance > fdt:   # if no wall forward, it means current error is cause by a gap
            angle *= ac
        if error > et:   # if error is quite big, even if no wall forward, we should turn
            angle /= ac 
        
        # TODO: fill in drive message and publish
        drive_msg = AckermannDriveStamped()
        steer_angle_threshold = self.get_parameter('steer_angle_threshold').get_parameter_value().integer_value
        angle = np.clip(angle, -steer_angle_threshold / 180 * np.pi, steer_angle_threshold / 180 * np.pi)
        
        speed = self.calculate_speed(angle)
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = angle
        self.get_logger().info("error:%f; speed:%f; angle:%f;" % (error, speed, angle))
        self.publisher_.publish(drive_msg)
        return angle, speed
        

    def calculate_speed(self, angle):
        """
        As the instruction says:
        If the steering angle is between 0 degrees and 10 degrees, the car should drive at 1.5 meters per second.
        If the steering angle is between 10 degrees and 20 degrees, the speed should be 1.0 meters per second.
        Otherwise, the speed should be 0.5 meters per second.
        """
        if np.abs(angle) < 10 / 180 * np.pi:
            speed = 2.0
        elif np.abs(angle) >= 10 / 180 * np.pi and np.abs(angle) < 20 / 180 * np.pi:
            speed = 1.0
        else:
            speed = 0.5
        return speed
        
        
    def scan_callback(self, msg:LaserScan):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
        ranges = msg.ranges
        dis = self.get_parameter('distance').get_parameter_value().double_value
        error = self.get_error(ranges, dis) # TODO: replace with error calculated by get_error()
        angle, speed = self.pid_control(self.get_range(ranges, 0), error) # TODO: actuate the car with PID

        # alpha = self.get_parameter('alpha').get_parameter_value().double_value
        # self.filtered_error = alpha * error + (1 - alpha) * self.filtered_error
        
        if self.plot_switch == "on":
            # Update data
            self.x_data.append(len(self.x_data))
            self.y_data_error.append(error)
            self.y_data_forward_dis.append(self.get_range(ranges, 0))
            self.y_data_angle.append(angle)
            self.y_data_speed.append(speed)

            # Update plot
            self.line_error.set_xdata(self.x_data)
            self.line_error.set_ydata(self.y_data_error)
            self.line_forward_dis.set_xdata(self.x_data)
            self.line_forward_dis.set_ydata(self.y_data_forward_dis)
            self.line_angle.set_xdata(self.x_data)
            self.line_angle.set_ydata(self.y_data_angle)
            self.line_speed.set_xdata(self.x_data)
            self.line_speed.set_ydata(self.y_data_speed)
            self.ax1.relim()
            self.ax1.autoscale_view()
            self.ax2.relim()
            self.ax2.autoscale_view()
            self.ax3.relim()
            self.ax3.autoscale_view()
            self.ax4.relim()
            self.ax4.autoscale_view()
            # Draw the updated plot
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            plt.show()
            

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    
    wall_follow_node = WallFollow()
    
    if wall_follow_node.plot_switch == "on":
        ani = FuncAnimation(wall_follow_node.fig, wall_follow_node.scan_callback, interval=0)  # Update every 100 milliseconds
    rclpy.spin(wall_follow_node)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()