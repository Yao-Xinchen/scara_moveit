#include "scara_motor_driver/motor_joint.hpp"

MotorJoint::MotorJoint(std::string motor, std::string joint, double offset, double ratio)
    : offset(offset), ratio(ratio)
{
    motor_name = std::move(motor);
    joint_name = std::move(joint);
}

[[nodiscard]] double MotorJoint::j2m_pos(double joint_pos) const
{
    return joint_pos * ratio + offset;
}

[[nodiscard]] double MotorJoint::m2j_pos(double motor_pos) const
{
    return (motor_pos - offset) / ratio;
}

[[nodiscard]] double MotorJoint::j2m_vel(double joint_vel) const
{
    return joint_vel * ratio;
}

[[nodiscard]] double MotorJoint::m2j_vel(double motor_vel) const
{
    return motor_vel / ratio;
}