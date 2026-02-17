#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class Number_PublisherNode(Node):
    def __init__(self):
        super().__init__("Number_Publisher")
        self.publisher_ = self.create_publisher(Int64, "number", 10) # data_type, topic, 10 messages is the queue size
        self.num = 5
        self.timer_ = self.create_timer(1, self.publish_news) # time between calls, call back function
        self.get_logger().info("Number Publisher initiated.")

    def publish_news(self):
        msg = Int64()
        msg.data = self.num
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Number_PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()