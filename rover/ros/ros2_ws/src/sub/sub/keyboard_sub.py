import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class KeyboardSubscriber(Node):
    def __init__(self):
        super().__init__('keyboard_subscriber')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)


    def listener_callback(self, msg):
        self.get_logger().info(f"Linear: {msg.linear.x}, Angular: {msg.angular.z}")





def main(args=None):
    rclpy.init(args=args)

    keyboard_sub = KeyboardSubscriber()

    rclpy.spin(keyboard_sub)

    keyboard_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()