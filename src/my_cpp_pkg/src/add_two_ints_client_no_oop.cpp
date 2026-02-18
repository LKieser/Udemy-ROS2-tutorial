#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop"); //node name
    
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints"); // service name
    while(!client->wait_for_service(1s)) // wait until connection is established
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for ther server...");
    }

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = 9;
    request->b = 7;

    auto future = client->async_send_request(request); // send request
    rclcpp::spin_until_future_complete(node, future); // spin until request is sent

    auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "%d + %d = %d", (int)request->a, (int)request->b, (int)response->sum);

    rclcpp::shutdown();
    return 0;
}