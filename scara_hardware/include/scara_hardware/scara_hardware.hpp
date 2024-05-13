#ifndef SCARA_HARDWARE_HPP
#define SCARA_HARDWARE_HPP

#include "scara_hardware/mj_map.hpp"
#include "string"
#include "unordered_map"
#include "vector"
#include <string>
#include <unordered_map>
#include <utility>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "device_interface/msg/motor_goal.hpp"
#include "device_interface/msg/motor_state.hpp"

using hardware_interface::return_type;

using namespace std;

namespace scara
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC ScaraHardware : public hardware_interface::SystemInterface
{
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    rclcpp::Subscription<device_interface::msg::MotorState>::SharedPtr motor_state_sub_;
    rclcpp::Publisher<device_interface::msg::MotorGoal>::SharedPtr motor_goal_pub_;

    MJMap mj_map;

    unordered_map<string, tuple<double, double>> joint_states;
    unordered_map<string, tuple<double, double>> joint_commands;

    void motor_state_callback(const device_interface::msg::MotorState::SharedPtr msg);
};

}

#endif  // SCARA_HARDWARE_HPP