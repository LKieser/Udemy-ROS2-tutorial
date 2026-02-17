#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node // inherit from rclcpp::Node
{
public:
    MyNode() : Node("cpp_test"), counter_(0) // build constructor for parent class and initialize counter
    {
        RCLCPP_INFO(this->get_logger(), ":))))))");
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&MyNode::timerCallBack, this));
    }
private:
    void timerCallBack()
    {
        RCLCPP_INFO(this->get_logger(), ":ppppppp");
        counter_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // initiazlize ros2 communications
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}