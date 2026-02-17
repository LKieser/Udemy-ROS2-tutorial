#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServerNode(Node):
    def __init__(self):
        super().__init__("add_two_ints")
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints) # Type, Name of service (should be a verb), callback

        self.get_logger().info("Add Two Ints server has been started")

    # this callback is called when a request is recieved by the server
    def callback_add_two_ints(self, request: AddTwoInts.Request, response: AddTwoInts.Response): # give type to each variable
        response.sum = request.a + request.b
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")
        return response # have to return response or you will get an exception



def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
