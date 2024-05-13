#ifndef MOTOR_JOINT_HPP
#define MOTOR_JOINT_HPP

#include <string>

class MotorJoint
{
public:
    MotorJoint() = default;

    MotorJoint(std::string motor,std::string joint, double offset, double ratio);

    [[nodiscard]] double j2m_pos(double joint_pos) const;

    [[nodiscard]] double m2j_pos(double motor_pos) const;

    [[nodiscard]] double j2m_vel(double joint_vel) const;

    [[nodiscard]] double m2j_vel(double motor_vel) const;

    std::string motor_name{};
    std::string joint_name{};
    double offset;
    double ratio;
};

#endif // MOTOR_JOINT_HPP