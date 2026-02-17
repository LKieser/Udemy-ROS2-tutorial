#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client_no_oop") # name of client

    client = node.create_client(AddTwoInts, "add_two_ints") # type, name of service (same as the server name)
    while not client.wait_for_service(1.0): # stays in loop until server is found. The timeout in the function is the time it waits before looping again
        node.get_logger().warn("Waiting for Add Two Ints server...")
    
    request = AddTwoInts.Request()
    request.a = 9
    request.b = 11

    future = client.call_async(request)             # send request
    rclpy.spin_until_future_complete(node, future)  # wait for response

    response = future.result()                      # set response once recieved
    node.get_logger().info(f"{request.a} + {request.b} = {response.sum}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
