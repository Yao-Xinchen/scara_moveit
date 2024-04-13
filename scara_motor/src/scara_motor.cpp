#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <rclcpp/publisher.hpp>
#include <string>
#include <unordered_map>

#include "scara_motor/motor.hpp"

#include "sensor_msgs/msg/joint_state.hpp" // produced by moveit
#include "motor_interface/msg/motor_goal.hpp"

#define NaN std::nan("")

using std::unordered_map, std::string;
using motor_interface::msg::MotorGoal, sensor_msgs::msg::JointState;

class ScaraMotor : public rclcpp::Node
{
public:
    ScaraMotor() : Node("scara_motor")
    {
        motor_goal_pub_ = this->create_publisher<MotorGoal>("motor_goal", 10);
        joint_state_sub_ = this->create_subscription<JointState>("joint_states", 10.0, [this](JointState::SharedPtr msg) {
            joint_state_callback(msg);
        });

        get_params();

        RCLCPP_INFO(this->get_logger(), "ScaraMotor initialized.");
    }

private:
    rclcpp::Subscription<JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<MotorGoal>::SharedPtr motor_goal_pub_;

    unordered_map<string, Motor> joint2motor = {
        {"joint1", Motor("J1", 0.0, 973.4)},
        {"joint2", Motor("J2", -1.22, 1.0)},
        {"joint3", Motor("J3", 0.28, 1.0)},
        {"joint4", Motor("J4", 0.0, 20.0)},
        {"joint5", Motor("J5", 0.0, 20.0)},
        {"joint6", Motor("J6", 0.0, 20.0)},
        {"joint7", Motor("J7", 0.0, 108.0)},
    };

    void joint_state_callback(JointState::SharedPtr state_msg_)
    {
        auto goal = empty_motor_goal();

        for (size_t i = 0; i < state_msg_->name.size(); i++)
        {
            const string& joint = state_msg_->name[i]; // e.g. "joint1"
            const double& pos = state_msg_->position[i]; // goal of joint position

            const auto& motor_id = joint2motor[joint].rid; // e.g. "J1"
            const auto& goal_pos = joint2motor[joint].calc_pos(pos); // goal of motor position

            goal.motor_id.push_back(motor_id);
            goal.goal_pos.push_back(goal_pos);
            goal.goal_vel.push_back(NaN);
            goal.goal_tor.push_back(NaN);
        }

        motor_goal_pub_->publish(goal);
    }

    void get_params()
    {
        for (auto &[joint, motor] : joint2motor)
        {
            double& offset = motor.offset;
            double& ratio = motor.ratio;
            std::string offset_name = "offset." + joint; // e.g. "offset.joint1"
            std::string ratio_name = "ratio." + joint;   // e.g. "ratio.joint1"
            offset = this->declare_parameter(offset_name, offset);
            ratio = this->declare_parameter(ratio_name, ratio);

            RCLCPP_INFO(this->get_logger(), "Motor %s: offset %.2f, ratio %.2f", joint.c_str(), offset, ratio);
        }
    }

    MotorGoal empty_motor_goal()
    {
        MotorGoal goal;
        goal.motor_id.clear();
        goal.goal_pos.clear();
        goal.goal_vel.clear();
        goal.goal_tor.clear();
        return goal;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScaraMotor>());
    rclcpp::shutdown();
    return 0;
}