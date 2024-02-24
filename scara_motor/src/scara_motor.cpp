#include "rclcpp/rclcpp.hpp"
#include <string>
#include <unordered_map>

using std::unordered_map, std::string;

unordered_map<string, string> joint2motor = {
    {"joint1", "J1"},
    {"joint2", "J2"},
    {"joint3", "J3"},
    {"joint4", "J4"},
    {"joint5", "J5"},
    {"joint6", "J6"},
    {"joint7", "J7"},
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::shutdown();
    return 0;
}