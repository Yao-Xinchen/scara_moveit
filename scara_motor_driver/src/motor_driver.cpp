#include "scara_motor_driver/motor_driver.hpp"
#include <cstddef>
#include <string>
#include <vector>

namespace Scara
{

CallbackReturn MotorDriver::on_init(const hardware_interface::HardwareInfo & info)
{
    // Call the base class implementation
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // Get the joint names
    uint count = node_->declare_parameter("joint_count", 0);
    vector<string> joint_names;
    joint_names = node_->declare_parameter("joint_names", joint_names);
    vector<string> motor_names;
    motor_names = node_->declare_parameter("motor_names", motor_names);
    vector<double> offsets;
    offsets = node_->declare_parameter("offsets", offsets);
    vector<double> ratios;
    ratios = node_->declare_parameter("ratios", ratios);
    for (size_t i = 0; i < count; i++)
    {
        mj_map.add_joint(motor_names[i], joint_names[i], offsets[i], ratios[i]);
    }

    // Check if the joint names match
    auto joint_names_set = mj_map.get_joint_names();
    if (info.joints.size() != joint_names_set.size())
    {
        RCLCPP_ERROR(node_->get_logger(), "Joint count mismatch.");
        return CallbackReturn::ERROR;
    }

    // Create a node and its publisher and subscriber
    node_ = rclcpp::Node::make_shared("scara_motor_driver_node");
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

PLUGINLIB_EXPORT_CLASS(Scara::MotorDriver, hardware_interface::SystemInterface)