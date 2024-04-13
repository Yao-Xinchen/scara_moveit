#include "scara_motor/motor.hpp"

Motor::Motor(std::string rid, double offset, double ratio)
{
    this->rid = rid;
    this->offset = offset;
    this->ratio = ratio;
}

double Motor::calc_pos(double pos) const
{
    return (pos + offset) * ratio;
}