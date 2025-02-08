#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import sys
import time

class RaspPublisher(Node):
    def __init__(self):
        super().__init__('rasp_publisher')
        self.publisher_ = self.create_publisher(Int32, 'rasp_control', 10)

        time.sleep(2)

    def publish_command(self, cmd):
        msg = Int32()
        msg.data = cmd
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent command: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = RaspPublisher()

    # 명령 인자
    if len(sys.argv) > 1:
        try:
            cmd = int(sys.argv[1]) # 명령 인자
            node.publish_command(cmd)
        except ValueError:
            node.get_logger().error("Invalid command value received.")
    else:
        node.get_logger().error("No command value received.")

    time.sleep(2)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
