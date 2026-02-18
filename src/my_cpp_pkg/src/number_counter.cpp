#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("NumberCounter"), the_count(0)
    {
        // topic section
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>( // type
            "number", 10,                                                       //  topic, queue
            std::bind(&NumberCounterNode::callbackNumberCounter, this, std::placeholders::_1));  // callback function, this, placeholder
        
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10); // topic, packages in queue
        RCLCPP_INFO(this->get_logger(), "Number Counter Initiated");

        // service section
        reset_counter_server_= this->create_service<example_interfaces::srv::SetBool>(
            "reset_counter",                                                         // service name
            std::bind(&NumberCounterNode::callbackResetCounter, this, _1, _2));    // bind function to instance (this->callback) _1  = request /  _2 = response
        RCLCPP_INFO(this->get_logger(), "Add Two Ints Service has been started");

    }

private:
    void callbackNumberCounter(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        the_count = msg->data + the_count;
        auto new_msg = example_interfaces::msg::Int64();
        new_msg.data = the_count;
        publisher_->publish(new_msg);
    }
    void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                              const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        if (request->data) {
            the_count = 0;
            response->success = true;
            response->message = "Counter has been reset";
        }
        else
        {
            response->success = false;
            response->message = "Counter has not been reset";
        }
    }
    void printAdd()
    {
        
    }
    std::int64_t the_count;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr reset_counter_server_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
