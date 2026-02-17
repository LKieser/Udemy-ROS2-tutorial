#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
from functools import partial # partial from functools lets me add extra arguments to built in functions


class AddTwoIntsNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.client = self.create_client(AddTwoInts, "add_two_ints") # Type, service name (same as server)

    def call_add_two_ints(self, a, b):
        while not self.client.wait_for_service(1.0): # stays in loop until server is found. 1 sec between checks
            self.get_logger().warn("Waiting for Add Two Ints server...")
        
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = self.client.call_async(request)    # send request
        future.add_done_callback(partial(self.callback_call_add_two_ints, request=request)) # will go to function when done
    
    def callback_call_add_two_ints(self, future, request):
        response = future.result()
        self.get_logger().info(f"{request.a} + {request.b} = {response.sum}")

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsNode()
    node.call_add_two_ints(a=2,b=50)
    node.call_add_two_ints(a=8,b=5)
    node.call_add_two_ints(a=3,b=3)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
