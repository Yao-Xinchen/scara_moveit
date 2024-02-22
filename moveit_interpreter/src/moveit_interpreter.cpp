#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>

using sensor_msgs::msg::JointState;

class MoveitInterpreter : public rclcpp::Node
{
public:
    MoveitInterpreter() : Node("moveit_interpreter")
    {
        joint_state_sub_ = this->create_subscription<JointState>(
            "joint_states", 10,
            std::bind(&MoveitInterpreter::joint_state_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;

    void joint_state_callback(const JointState::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg->name[0].c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}