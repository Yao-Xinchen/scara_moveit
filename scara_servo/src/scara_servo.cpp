#include "rclcpp/rclcpp.hpp"
#include <memory>

#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/subscription.hpp>

#include "behavior_interface/msg/end_vel.hpp"

using control_msgs::msg::JointJog;
using geometry_msgs::msg::TwistStamped;
using behavior_interface::msg::EndVel;

#define EEF_FRAME_ID "link8" // end effector frame
#define BASE_FRAME_ID "base_link"

class ScaraServo : public rclcpp::Node
{
public:
    ScaraServo() : Node("scara_servo")
    {
        // joint_pub_ = this->create_publisher<JointJog>("joint_jog", 10);
        twist_pub_ = this->create_publisher<TwistStamped>("cartesian_velocity", 10);

        end_vel_sub_ = this->create_subscription<EndVel>("end_vel", 10,
            [this](EndVel::SharedPtr msg) {
                end_vel_callback(msg);
        });

        RCLCPP_INFO(this->get_logger(), "ScaraServo initialized.");
    }

    void init()
    {
        this->servo_ = init_servo();
        this->servo_->start();
    }

private:
    // rclcpp::Publisher<JointJog>::SharedPtr joint_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr twist_pub_;
    std::shared_ptr<moveit_servo::Servo> servo_;
    
    rclcpp::Subscription<EndVel>::SharedPtr end_vel_sub_;

    std::shared_ptr<moveit_servo::Servo> init_servo()
    {
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            this->shared_from_this(), "robot_description", tf_buffer, "planning_scene_monitor");
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

        auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(
            this->shared_from_this()/*, this->get_logger()*/
        );
        if (!servo_parameters) RCLCPP_FATAL(this->get_logger(), "Failed to load the servo parameters");

        auto _servo_ = std::make_shared<moveit_servo::Servo>(this->shared_from_this(), servo_parameters, planning_scene_monitor_);
        if (!_servo_) RCLCPP_FATAL(this->get_logger(), "Failed to initialize the servo");

        return _servo_;
    }

    void end_vel_callback(const EndVel::SharedPtr msg)
    {
        TwistStamped::UniquePtr twist_msg = std::make_unique<TwistStamped>();
        twist_msg->header.stamp = this->now();
        twist_msg->header.frame_id = EEF_FRAME_ID;
        twist_msg->twist.linear.x = msg->x;
        twist_msg->twist.linear.y = msg->y;
        twist_msg->twist.linear.z = msg->z;
        twist_msg->twist.angular.x = msg->roll;
        twist_msg->twist.angular.y = msg->pitch;
        twist_msg->twist.angular.z = msg->yaw;
        twist_pub_->publish(std::move(twist_msg));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::sleep_for(std::chrono::seconds(1)); // wait for rviz to start
    auto node_ = std::make_shared<ScaraServo>();
    node_->init();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}