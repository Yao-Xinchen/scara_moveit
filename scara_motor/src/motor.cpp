#include "scara_motor/motor.hpp"

Motor::Motor(std::string rid, double offset, double ratio)
{
    this->rid = rid;
    this->offset = offset;
    this->ratio = ratio;
}

double Motor::calc_pos(double joint_pos) const
{
    return (joint_pos + offset) * ratio;
}