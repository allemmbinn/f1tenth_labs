import rclpy
from rclpy.node import Node

import numpy as np
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        self.subscription_scan = self.create_subscription(LaserScan,lidarscan_topic,self.scan_callback,10)
        self.publisher_drive = self.create_publisher(AckermannDriveStamped,drive_topic, 10)
        # TODO: tune PID gains
        self.kp = 2.5
        self.kd = 0.
        self.ki = 0.

        self.integral = 0.
        self.prev_error = 0.
        self.error = 0.

        # Projected length parameters
        self.projected_length = 0.25 # TODO: Find a suitable value for this
        self.desired_value = 1.5 # TODO : Find the ideal value for this distance, this from  
        # The angle parameters
        self.angle_min = 0 
        self.angle_max = 0
        self.angle_increment =0

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        """
        index = (angle - self.angle_min) * len(range_data)/(self.angle_max - self.angle_min)
        range = range_data[index]
        return range

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """
        theta = 70 # TODO: Define and change and check 
        theta_radians = math.radians(theta)
        a = self.get_range(range_data, theta_radians)
        b = self.get_range(range_data, 0)
        alpha = math.atan((a * math.cos(theta_radians) - b)/(a * math.sin(theta_radians)))
        AB = b * math.cos(alpha)
        CD = AB + self.projected_length * math.sin(alpha)
        error = dist - CD
        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = self.kp * error + self.ki * self.integral + self.kd * (error - self.prev_error)
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = velocity
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = angle
        
        self.publisher_drive.publish(drive_msg) #To publish onto the drive topic

        self.prev_error = error
        self.integral += error

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        range_data = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment

        error = self.get_error(range_data,self.desired_value)
        error_abs = math.abs(error)
        if error_abs >= 0 and error_abs < 10:
            velocity = 1.5
        elif error_abs >= 10 and error_abs < 20:
            velocity = 1.
        else:
            velocity = 0.5
        self.pid_control(error, velocity)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()