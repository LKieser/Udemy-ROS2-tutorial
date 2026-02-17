#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node):
    def __init__(self):
        super().__init__("RobotNewsStation") 
        self.publisher_ = self.create_publisher(String, "robot_news", 10) # data_type, topic, 10 messages is the queue size
        self.robot_name = "C3PO"
        self.timer_ = self.create_timer(0.5, self.publish_news) # time between calls, call back function
        self.get_logger().info("Robot News Station has been started.")

    def publish_news(self):
        msg = String()
        msg.data  = f"Hi, this is {self.robot_name} from the robot news station."
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
