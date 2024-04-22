#include "scara_motor_driver/motor_driver.hpp"

namespace Scara
{

CallbackReturn MotorDriver::on_init(const hardware_interface::HardwareInfo & info)
{
    return CallbackReturn::SUCCESS;
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