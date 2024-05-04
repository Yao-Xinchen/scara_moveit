#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char * argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Next step goes here
        // Create the MoveIt MoveGroup Interface
        using moveit::planning_interface::MoveGroupInterface;
        auto move_group_interface = MoveGroupInterface(node, "scara_arm");
        move_group_interface.setMaxVelocityScalingFactor(1.0);

        // Set a target Pose
        auto const target_pose = []{
            geometry_msgs::msg::Pose msg;
            msg.orientation.x = 0.009;
            msg.orientation.y = 0.718;
            msg.orientation.z = 0.696;
            msg.orientation.w = -0.025;
            double magnitude = std::sqrt(msg.orientation.x * msg.orientation.x +
                                 msg.orientation.y * msg.orientation.y +
                                 msg.orientation.z * msg.orientation.z +
                                 msg.orientation.w * msg.orientation.w);
            msg.orientation.x /= magnitude;
            msg.orientation.y /= magnitude;
            msg.orientation.z /= magnitude;
            msg.orientation.w /= magnitude;
            tf2::Quaternion q(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            );
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            RCLCPP_INFO(rclcpp::get_logger("log"), "Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
            
            msg.position.x = 0.372;
            msg.position.y = 0.348;
            msg.position.z = 0.326;
            return msg;
        }();
        move_group_interface.setPoseTarget(target_pose);

        // Create a plan to that target pose
        auto const [success, plan] = [&move_group_interface]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface.plan(msg));
            return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if(success) {
            move_group_interface.execute(plan);
        } else {
            RCLCPP_ERROR(logger, "Planing failed!");
        }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}