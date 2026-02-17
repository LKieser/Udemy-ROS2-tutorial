#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

using namespace std::chrono_literals;

class NumberCounterNode : public rclcpp::Node
{
public:
    NumberCounterNode() : Node("NumberCounter"), the_count(0)
    {
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>( // type
            "number", 10,                                                       //  topic, queue
            std::bind(&NumberCounterNode::callbackNumberCounter, this, std::placeholders::_1));  // callback function, this, placeholder
        
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10); // topic, packages in queue
        RCLCPP_INFO(this->get_logger(), "Number Counter Initiated");
    }

private:
    void callbackNumberCounter(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        the_count = msg->data + the_count;
        auto new_msg = example_interfaces::msg::Int64();
        new_msg.data = the_count;
        publisher_->publish(new_msg);
    }
    void printAdd()
    {
        
    }
    std::int64_t the_count;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
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
