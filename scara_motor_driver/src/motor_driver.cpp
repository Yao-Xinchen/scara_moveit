#include "scara_motor_driver/motor_driver.hpp"
#include <cstddef>
#include <string>
#include <sys/types.h>
#include <vector>

#define YAML flase

namespace scara
{

CallbackReturn MotorDriver::on_init(const hardware_interface::HardwareInfo & info)
{
    // Call the base class implementation
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Get the joint names
    node_ = rclcpp::Node::make_shared("scara_motor_driver_node");
    vector<string> joint_names;
    vector<string> motor_names;
    vector<double> offsets;
    vector<double> ratios;
#if YAML == false
    uint count = 7;
    joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"};
    motor_names = {"J1", "J2", "J3", "J4", "J5", "J6", "J7"};
    offsets = {0.0, -1.28, -2.71, 1.571, 0.0, 0.0, -1.571};
    ratios = {973.4, 1.0, 1.0, 20.0, 20.0, 20.0, 108.0};
#else
    // uint count = node_->declare_parameter("joint_count", 0);
    // joint_names = node_->declare_parameter("joint_names", joint_names);
    // motor_names = node_->declare_parameter("motor_names", motor_names);
    // offsets = node_->declare_parameter("offsets", offsets);
    // ratios = node_->declare_parameter("ratios", ratios);
#endif

    // Add the joints to the map
    for (size_t i = 0; i < count; i++)
    {
        mj_map.add_joint(motor_names[i], joint_names[i], offsets[i], ratios[i]);
    }

    // get ready for the joint states and commands
    for (const auto& joint : joint_names)
    {
        joint_states[joint] = {0.0, 0.0};
        joint_commands[joint] = {0.0, 0.0};
    }

    // Check if the joint names match
    auto joint_names_set = mj_map.get_joint_names();
    for (auto joint : info.joints)
    {
        if (joint_names_set.find(joint.name) == joint_names_set.end())
        {
            RCLCPP_ERROR(node_->get_logger(), "Joint %s not found in the joint map", joint.name.c_str());
            return CallbackReturn::ERROR;
        }
    }

    // Create a node and its publisher and subscriber
    motor_state_sub_ = node_->create_subscription<motor_interface::msg::MotorState>(
        "motor_state", 10, std::bind(&MotorDriver::motor_state_callback, this, std::placeholders::_1));
    motor_goal_pub_ = node_->create_publisher<motor_interface::msg::MotorGoal>("motor_goal", 10);

    // Done
    RCLCPP_INFO(node_->get_logger(), "ScaraMotorDriver initialized.");
    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorDriver::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto& [joint, state]: joint_states)
    {
        auto& [pos, vel] = state;
        state_interfaces.emplace_back(joint, "position", &pos);
        state_interfaces.emplace_back(joint, "velocity", &vel);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorDriver::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto& [joint, command]: joint_commands)
    {
        auto& [pos, vel] = command;
        command_interfaces.emplace_back(joint, "position", &pos);
        command_interfaces.emplace_back(joint, "velocity", &vel);
    }
    return command_interfaces;
}

return_type MotorDriver::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // update the joint states
    // already done in the callback
    return return_type::OK;
}

return_type MotorDriver::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    // publish the joint commands
    motor_interface::msg::MotorGoal msg;
    for (const auto& [joint, command]: joint_commands)
    {
        auto [pos, vel] = command;
        auto motor = mj_map.j2m_name(joint);
        msg.motor_id.push_back(motor);
        // Check if pos and vel are both not NaN
        if (!std::isnan(pos) && !std::isnan(vel))
            // If both are not NaN, set vel to NaN
            vel = std::nan("");

        msg.goal_pos.push_back(mj_map.j2m_pos(joint, pos));
        msg.goal_vel.push_back(mj_map.j2m_vel(joint, vel));
        msg.goal_tor.push_back(std::nan(""));
    }
    motor_goal_pub_->publish(msg);
    return return_type::OK;
}

void MotorDriver::motor_state_callback(const motor_interface::msg::MotorState::SharedPtr msg)
{
    // Update the joint states
    for (size_t i = 0; i < msg->motor_id.size(); i++)
    {
        auto& motor = msg->motor_id[i];
        auto joint = mj_map.m2j_name(motor);
        auto joint_pos = mj_map.m2j_pos(motor, msg->present_pos[i]);
        auto joint_vel = mj_map.m2j_vel(motor, msg->present_vel[i]);
        this->joint_states[joint] = {joint_pos, joint_vel};
    }
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(scara::MotorDriver, hardware_interface::SystemInterface)