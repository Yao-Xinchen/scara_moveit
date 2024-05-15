#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rclcpp/subscription.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "geometry_msgs/msg/vector3.hpp"

class ScaraObjects : public rclcpp::Node
{
public:
    ScaraObjects() : Node("scara_objects")
    {
        auto const logger = this->get_logger();

        // rclcpp::executors::SingleThreadedExecutor executor;
        // executor.add_node(node);
        // auto spinner = std::thread([&executor]() { executor.spin(); });

        // using moveit::planning_interface::MoveGroupInterface;
        // move_group_interface = std::make_shared<MoveGroupInterface>(this->shared_from_this(),"scara_arm");

        obj_sub = this->create_subscription<geometry_msgs::msg::Vector3>("ore_position", 10,
            [this](geometry_msgs::msg::Vector3::SharedPtr msg) {
                obj_callback(msg);
        });

        RCLCPP_INFO(logger, "ScaraObjects initialized.");
    }

    void init()
    {
        first = true;
        using moveit::planning_interface::MoveGroupInterface;
        move_group_interface = std::make_unique<MoveGroupInterface>(this->shared_from_this(),"scara_arm");
        RCLCPP_INFO(this->get_logger(), "ScaraObjects started.");
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr obj_sub;

    bool first;
    geometry_msgs::msg::Vector3 object_position;

    void obj_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        if (!first) return;

        auto logger = this->get_logger();

        object_position = *msg;
        first = false;

        auto frame_id = move_group_interface->getPlanningFrame();
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "ore";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.2;
        primitive.dimensions[primitive.BOX_Y] = 0.2;
        primitive.dimensions[primitive.BOX_Z] = 0.2;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = object_position.z - 0.58;
        box_pose.position.y = object_position.x;
        box_pose.position.z = object_position.y + 0.5;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        planning_scene_interface.applyCollisionObject(collision_object);

        // attach
        geometry_msgs::msg::Pose relative_pose;
        relative_pose.position.x = -0.05;
        relative_pose.position.y = 0.0;
        relative_pose.position.z = 0.15;
        tf2::Quaternion q;
        q.setRPY(M_PI_2, - M_PI_2, M_PI);
        relative_pose.orientation = tf2::toMsg(q);
        RCLCPP_INFO(logger, "Relative pose: x=%f, y=%f, z=%f", relative_pose.position.x,
                    relative_pose.position.y, relative_pose.position.z);
        RCLCPP_INFO(logger, "Relative pose: qx=%f, qy=%f, qz=%f, qw=%f", relative_pose.orientation.x,
                    relative_pose.orientation.y, relative_pose.orientation.z, relative_pose.orientation.w);

        geometry_msgs::msg::PoseStamped relative_pose_stamped;
        relative_pose_stamped.header.frame_id = collision_object.id;
        relative_pose_stamped.pose = relative_pose;
        geometry_msgs::msg::PoseStamped target_pose_stamped;
        // Convert the Pose to a TransformStamped
        geometry_msgs::msg::TransformStamped transform;
        transform.header.frame_id = collision_object.id;
        transform.header.stamp = rclcpp::Clock().now();
        transform.transform.translation.x = collision_object.primitive_poses[0].position.x;
        transform.transform.translation.y = collision_object.primitive_poses[0].position.y;
        transform.transform.translation.z = collision_object.primitive_poses[0].position.z;
        transform.transform.rotation = collision_object.primitive_poses[0].orientation;
        tf2::doTransform(relative_pose_stamped, target_pose_stamped, transform);

        move_group_interface->setPoseTarget(target_pose_stamped.pose);

        // Create a plan to that target pose
        auto const [success, plan] = [this] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group_interface->plan(msg));
            return std::make_pair(ok, msg);
        }();

        // Execute the plan
        if (success)
        {
            move_group_interface->execute(plan);
            // Attach the collision object to the end effector
            std::vector<std::string> touch_links;
            touch_links.push_back("link7");
            move_group_interface->attachObject(collision_object.id, "link7", touch_links);
            RCLCPP_INFO(logger, "Object attached to the end effector!");
        }
        else
        {
            RCLCPP_ERROR(logger, "Planing failed!");
        }
        RCLCPP_INFO(logger, "Planing succeeded!");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node_ = std::make_shared<ScaraObjects>();
    node_->init();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}