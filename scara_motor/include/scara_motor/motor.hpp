#ifndef MOTOR_HPP
#define MOTOR_HPP

#include <string>

class Motor
{
public:
    Motor() = default;

    Motor(std::string rid, double offset, double ratio);

    [[nodiscard]] double calc_pos(double joint_pos) const;

    std::string rid;
    double offset;
    double ratio;
};

#endif // MOTOR_HPP