#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import SetLed


class BatteryNode(Node):
    def __init__(self):
        super().__init__("Battery_node")
        # create the client that will send to the LED panel the battery status
        self.battery_status_client = self.create_client(SetLed, "set_led")
        self.battery_state = 1 # 0 is low, 1 is full
        self.time_counter = 0
        # 4 seconds for the battery to drain and 6 seconds to fill. After either one send a request
        self.battery_change_timer = self.create_timer(1.0, self.check_battery_state)

    # this is a good way to get the current time. This function was not used in this program
    def get_current_time_seconds(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1000000000.0 # return the time with nanoseconds appended. divide by 1.0 x 10^9

    def check_battery_state(self):
        self.time_counter += 1

        # empty battery
        if self.time_counter == 4 and self.battery_state:
            self.time_counter = 0
            self.battery_state = 0 # toggle battery_state
            self.call_set_led(True)

        # full battery
        if self.time_counter == 6 and not self.battery_state:
            self.time_counter = 0
            self.battery_state = 1 # toggle battery_state
            self.call_set_led(False)


    def call_set_led(self, b_state):
        # stay in loop until connected to server
        while not self.battery_status_client.wait_for_service(1.0):
            self.get_logger().info("Waiting for set_led server...")
        
        # set up variable that will carry the request message
        request = SetLed.Request()
        # set battery_low to true if the battery is empty and false if the battery is full
        request.battery_low = b_state

        future = self.battery_status_client.call_async(request) # send the request
        future.add_done_callback(self.get_response_from_led_panel) # when done go to call back
        

    def get_response_from_led_panel(self, future):
        response = future.result() # print the response from the request
        self.get_logger().info(f"{response.message}\nIs Led on: {response.led_on}")



def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
