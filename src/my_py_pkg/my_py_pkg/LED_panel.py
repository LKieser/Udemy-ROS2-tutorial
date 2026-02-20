#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import LedPanelState
from my_robot_interfaces.srv import SetLed


class LedPanelNode(Node):
    def __init__(self):
        super().__init__("LED_panel_node")
        # create publisher and timer for the publisher
        self.led_state_publisher = self.create_publisher(LedPanelState, "led_panel_state", 10)
        self.timer = self.create_timer(1.0, self.publish_led_state) # if you want to call publish_publish_led_state at other times you can just call the function anywhere
        # create server to host service
        self.led_control_server = self.create_service(SetLed, "set_led", self.callback_set_led)
        
        # create variable
        self.battery_is_low = False
        self.get_logger().info("Led Panel Node initiated")

    def publish_led_state(self):
        msg = LedPanelState()
        msg.led_status[0] = 0
        msg.led_status[1] = 0
        msg.led_status[2] = 1 if self.battery_is_low else 0 
        self.led_state_publisher.publish(msg)

    def callback_set_led(self, request: SetLed.Request, response: SetLed.Response):
        if request.battery_low:
            self.battery_is_low = True
            response.led_on = True
            response.message = "Led has been turned on"
        else:
            self.battery_is_low = False
            response.led_on = False
            response.message = "Led has been turned off"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LedPanelNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()