#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <string>
#include <unordered_map>

#include "sensor_msgs/msg/joint_state.hpp" // produced by moveit
#include "motor_interface/msg/motor_goal.hpp"

using std::unordered_map, std::string;
using motor_interface::msg::MotorGoal, sensor_msgs::msg::JointState;

unordered_map<string, string> joint2motor = {
    {"joint1", "J1"},
    {"joint2", "J2"},
    {"joint3", "J3"},
    {"joint4", "J4"},
    {"joint5", "J5"},
    {"joint6", "J6"},
    {"joint7", "J7"},
};

unordered_map<string, float> motor_offset = {
    {"J1", -1.22},
    {"J2", -0.69},
    {"J3", 0.0},
    {"J4", 0.0},
    {"J5", 0.0},
    {"J6", 0.0},
    {"J7", 0.0},
};

class ScaraMotor : public rclcpp::Node
{
public:
    ScaraMotor() : Node("scara_motor")
    {
        joint_state_sub_ = this->create_subscription<JointState>("joint_states", 10, [this](JointState::SharedPtr msg) {
            joint_state_callback(msg);
        });
        motor_goal_pub_ = this->create_publisher<MotorGoal>("motor_goal", 10);
    }

private:
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<MotorGoal>::SharedPtr motor_goal_pub_;

    void joint_state_callback(JointState::SharedPtr state_msg_)
    {
        MotorGoal motor_goal_msg;
        motor_goal_msg.motor_id.clear();
        motor_goal_msg.goal_pos.clear();
        motor_goal_msg.goal_vel.clear();

        for (size_t i = 0; i < state_msg_->name.size(); i++)
        {
            auto joint = state_msg_->name[i];
            auto position = state_msg_->position[i];
            auto motor_rid = joint2motor[joint];
            auto offset = motor_offset[motor_rid];

            motor_goal_msg.motor_id.push_back(motor_rid);
            motor_goal_msg.goal_pos.push_back(position + offset);
            motor_goal_msg.goal_vel.push_back(0.0);
        }
        motor_goal_pub_->publish(motor_goal_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScaraMotor>());
    rclcpp::shutdown();
    return 0;
}