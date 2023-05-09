import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped

class Relay(Node):

    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(AckermannDriveStamped,'drive',self.listener_callback,10)
        
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        msgRelay = AckermannDriveStamped()
        msgRelay.drive.speed = msg.drive.speed * 3
        msgRelay.drive.steering_angle = msg.drive.steering_angle * 3
        msgRelay.header = msg.header
        self.publisher_.publish(msgRelay)  



def main(args=None):
    rclpy.init(args=args)

    relay = Relay()

    rclpy.spin(relay)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    relay.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()