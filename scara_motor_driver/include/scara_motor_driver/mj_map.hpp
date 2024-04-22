#ifndef MJ_MAP_HPP
#define MJ_MAP_HPP

#include <unordered_map>
#include <string>
#include <memory>
#include <set>

#include "motor_joint.hpp"

using std::string;
using std::shared_ptr;
using std::unordered_map;

class MJMap
{
private:
    unordered_map<string, shared_ptr<MotorJoint>> joint_map{};
    unordered_map<string, shared_ptr<MotorJoint>> motor_map{};

public:
    MJMap() = default;

    void add_joint(string motor, string joint, double offset, double ratio);

    double j2m_pos(string joint, double joint_pos) const;

    double m2j_pos(string motor, double motor_pos) const;

    double j2m_vel(string joint, double joint_vel) const;

    double m2j_vel(string motor, double motor_vel) const;

    [[nodiscard]] std::set<string> get_joint_names() const;
};

#endif  // MJ_MAP_HPP