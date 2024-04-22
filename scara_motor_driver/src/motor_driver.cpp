#include "scara_motor_driver/motor_driver.hpp"

namespace Scara
{

CallbackReturn MotorDriver::on_init(const hardware_interface::HardwareInfo & info)
{
    // Call the base class implementation
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    node_ = rclcpp::Node::make_shared("scara_motor_driver_node");

    // Get the joint names
    
}

std::vector<hardware_interface::StateInterface> MotorDriver::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MotorDriver::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    return command_interfaces;
}

return_type MotorDriver::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return return_type::OK;
}

return_type MotorDriver::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    return return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(Scara::MotorDriver, hardware_interface::SystemInterface)