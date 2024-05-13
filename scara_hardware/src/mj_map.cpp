#include "scara_hardware/mj_map.hpp"

void MJMap::add_joint(string motor, string joint, double offset, double ratio)
{
    auto mj = make_shared<MotorJoint>(motor, joint, offset, ratio);
    joint_map[joint] = mj;
    motor_map[motor] = mj;
}

string MJMap::j2m_name(string joint) const
{
    return joint_map.at(joint)->motor_name;
}

string MJMap::m2j_name(string motor) const
{
    return motor_map.at(motor)->joint_name;
}

double MJMap::j2m_pos(string joint, double joint_pos) const
{
    return joint_map.at(joint)->j2m_pos(joint_pos);
}

double MJMap::m2j_pos(string motor, double motor_pos) const
{
    return motor_map.at(motor)->m2j_pos(motor_pos);
}

double MJMap::j2m_vel(string joint, double joint_vel) const
{
    return joint_map.at(joint)->j2m_vel(joint_vel);
}

double MJMap::m2j_vel(string motor, double motor_vel) const
{
    return motor_map.at(motor)->m2j_vel(motor_vel);
}

std::set<string> MJMap::get_joint_names() const
{
    std::set<string> joint_names;
    for (const auto& [joint, _] : joint_map)
    {
        joint_names.insert(joint);
    }
    return joint_names;
}