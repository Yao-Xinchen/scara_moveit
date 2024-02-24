#include "rclcpp/rclcpp.hpp"
#include <control_msgs/msg/detail/joint_jog__struct.hpp>
#include <memory>

#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using control_msgs::msg::JointJog;
using geometry_msgs::msg::TwistStamped;

#define EEF_FRAME_ID "link8" // end effector frame
#define BASE_FRAME_ID "base_link"

class ScaraServo : public rclcpp::Node
{
public:
    ScaraServo() : Node("scara_servo")
    {
        // joint_pub_ = this->create_publisher<JointJog>("joint_jog", 10);
        twist_pub_ = this->create_publisher<TwistStamped>("cartesian_velocity", 10);

        this->servo_ = init_servo();
        this->servo_->start();

        // joint_msg_ = std::make_unique<JointJog>();
        twist_msg_ = std::make_unique<TwistStamped>();
        twist_msg_->header.frame_id = EEF_FRAME_ID;
    }

private:
    // rclcpp::Publisher<JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;
    JointJog::UniquePtr joint_msg_;
    TwistStamped::UniquePtr twist_msg_;
    // MY_TODO: Subscription
    std::shared_ptr<moveit_servo::Servo> servo_;

    std::shared_ptr<moveit_servo::Servo> init_servo()
    {
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            this, "robot_description", tf_buffer, "planning_scene_monitor");
        if (planning_scene_monitor_->getPlanningScene())
        {
            planning_scene_monitor_->startStateMonitor("/joint_states");
            planning_scene_monitor_->setPlanningScenePublishingFrequency(25);
            planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                                "/moveit_servo/publish_planning_scene");
            planning_scene_monitor_->startSceneMonitor();
            planning_scene_monitor_->providePlanningSceneService();
        }
        else RCLCPP_ERROR(this->get_logger(), "Planning scene not configured");

        auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(this->shared_from_this());
        if (!servo_parameters) RCLCPP_FATAL(this->get_logger(), "Failed to load the servo parameters");

        auto _servo_ = std::make_shared<moveit_servo::Servo>(this->shared_from_this(), planning_scene_monitor_, servo_parameters);
        if (!_servo_) RCLCPP_FATAL(this->get_logger(), "Failed to initialize the servo");

        return _servo_;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::sleep_for(std::chrono::seconds(2)); // wait for rviz to start
    rclcpp::spin(std::make_shared<ScaraServo>());
    rclcpp::shutdown();
    return 0;
}