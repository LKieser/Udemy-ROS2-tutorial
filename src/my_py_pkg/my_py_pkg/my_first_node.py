#!/user/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node): # let's us inherit everything from Node
    def __init__(self):
        super().__init__("py_test")
        self.counter_ = 0
        self.get_logger().info("What's up!!")
        self.create_timer(1.0, self.timer_callback) # every 1 second call timer call back function

    def timer_callback(self):
        self.get_logger().info("Hiiiiii" + str(self.counter_))
        self.get_logger().info(f"Weeeee {self.counter_}")
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args) # initialize ros2
    node = MyNode() # create node from constructor
    rclpy.spin(node) # keeps node alive until i press ctrl C
    rclpy.shutdown() # shutdown node

if __name__ == "__main__":
    main()