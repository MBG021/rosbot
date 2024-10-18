#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty

MOVE_BINDINGS = {
    'w': (1, 0, 0, 0),
    's': (-1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    'q': (1, 0, 0, 1),
    'e': (1, 0, 0, -1),
    'z': (-1, 0, 0, 1),
    'c': (-1, 0, 0, -1)
}

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher_ = self.create_publisher(TwistStamped, '/ackermann_steering_controller/reference', 10)
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        self.get_logger().info("""
                               Control Your Robot!
---------------------------
Moving around:
   q    w    e
   a         d
   z    s    c

CTRL-C to quit
                               """)

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                twist_stamped = TwistStamped()

                if key in MOVE_BINDINGS:
                    linear_x, linear_y, linear_z, angular_z = MOVE_BINDINGS[key]
                    twist_stamped.header.stamp = self.get_clock().now().to_msg()
                    twist_stamped.twist.linear.x = self.linear_speed * linear_x
                    twist_stamped.twist.angular.z = self.angular_speed * angular_z
                elif key == ' ':
                    twist_stamped.twist.linear.x = 0.0
                    twist_stamped.twist.angular.z = 0.0
                elif key == '\x03':  # Ctrl+C
                    break

                self.publisher_.publish(twist_stamped)

        except Exception as e:
            self.get_logger().error(f"Error en la teleoperación: {e}")

        finally:
            twist_stamped = TwistStamped()
            twist_stamped.twist.linear.x = 0.0
            twist_stamped.twist.angular.z = 0.0
            self.publisher_.publish(twist_stamped)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


