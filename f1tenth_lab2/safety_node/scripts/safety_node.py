#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        self.subscription_scan = self.create_subscription(LaserScan,'scan',self.scan_callback,10)
        self.subscription_odom = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.publisher_drive = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.

        self.subscription_odom
        self.subscription_scan

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x


    def scan_callback(self, scan_msg):
        iTTC = np.array(scan_msg.ranges, dtype=np.float32)
        angles = np.arrange(scan_msg.angle_min,scan_msg.angle_max,scan_msg.angle_increment)
        range_rate = self.speed * np.cos(angles)
        range_rate[range_rate < 0] = 0
        iTTC = iTTC / range_rate
        ackermann_msg = AckermannDriveStamped()
        t = 0.3 # TODO: Tune the speed 
        if(np.nanmin(iTTC)) < t :
            ackermann_msg.drive.speed = 0.
            self.publisher_drive.publish(ackermann_msg)
        


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