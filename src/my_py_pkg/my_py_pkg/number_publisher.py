#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter # add this for the parameter callback
from example_interfaces.msg import Int64


class Number_PublisherNode(Node):
    def __init__(self):
        super().__init__("Number_Publisher")
        # create parameters
        self.declare_parameter("number", 2) # name: "number", default value: 2, type: int
        self.declare_parameter("timer_period", 1.0) # name: "timer_period", default value: 1.0, type: float
        # get parameters
        self.num = self.get_parameter("number").value
        self.timer_period = self.get_parameter("timer_period").value
        # parameter call back
        self.add_post_set_parameters_callback(self.parameters_callback)

        self.publisher_ = self.create_publisher(Int64, "number", 10) # interface, topic, 10 messages is the queue size
        self.timer_ = self.create_timer(self.timer_period, self.publish_news) # time between calls, call back function
        self.get_logger().info("Number Publisher initiated.")

    def publish_news(self):
        msg = Int64()
        msg.data = self.num
        self.publisher_.publish(msg)

    # need this callback if you want to be able to set the parameter to a new value in the terminal
    def parameters_callback(self, params: list[Parameter]):
        for param in params:
            if param.name == "number":
                self.num = param.value

def main(args=None):
    rclpy.init(args=args)
    node = Number_PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()