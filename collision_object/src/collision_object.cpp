#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

int main(int argc, char* argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
        "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // We spin up a SingleThreadedExecutor for the current state monitor to get
    // information about the robot's state.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "scara_arm");

    // Create collision object for the robot to avoid
    auto collision_object = [frame_id = move_group_interface.getPlanningFrame()] {
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
        box_pose.position.x = 0.4;
        box_pose.position.y = 0.5;
        box_pose.position.z = 0.3;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
    }();

    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);

    // moveit_msgs::msg::CollisionObject object_to_attach;
    // object_to_attach.id = "cylinder";

    // shape_msgs::msg::SolidPrimitive cylinder_primitive;
    // cylinder_primitive.type = cylinder_primitive.CYLINDER;
    // cylinder_primitive.dimensions.resize(2);
    // cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_HEIGHT] = 0.20;
    // cylinder_primitive.dimensions[cylinder_primitive.CYLINDER_RADIUS] = 0.04;

    // // We define the frame/pose for this cylinder so that it appears in the gripper.
    // object_to_attach.header.frame_id = move_group_interface.getEndEffectorLink();
    // geometry_msgs::msg::Pose grab_pose;
    // grab_pose.orientation.w = 1.0;
    // grab_pose.position.x = -0.08;
    // grab_pose.position.z = 0.0;

    // // First, we add the object to the world (without using a vector).
    // object_to_attach.primitives.push_back(cylinder_primitive);
    // object_to_attach.primitive_poses.push_back(grab_pose);
    // object_to_attach.operation = object_to_attach.ADD;
    // planning_scene_interface.applyCollisionObject(object_to_attach);

    // // attach the object to the robot
    // std::vector<std::string> touch_links;
    // touch_links.push_back("link7");
    // move_group_interface.attachObject(object_to_attach.id, "link7", touch_links);

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

    // Transform the relative pose to the frame of the object
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

    // Transform the relative pose to the frame of the object
    tf2::doTransform(relative_pose_stamped, target_pose_stamped, transform);
    RCLCPP_INFO(logger, "Target pose in the object frame: x=%f, y=%f, z=%f",
                target_pose_stamped.pose.position.x, target_pose_stamped.pose.position.y,
                target_pose_stamped.pose.position.z);
    RCLCPP_INFO(logger, "Target pose in the object frame: qx=%f, qy=%f, qz=%f, qw=%f",
                target_pose_stamped.pose.orientation.x, target_pose_stamped.pose.orientation.y,
                target_pose_stamped.pose.orientation.z, target_pose_stamped.pose.orientation.w);

    // Set the target pose
    move_group_interface.setPoseTarget(target_pose_stamped.pose);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success)
    {
        move_group_interface.execute(plan);
        // Attach the collision object to the end effector
        std::vector<std::string> touch_links;
        touch_links.push_back("link7");
        move_group_interface.attachObject(collision_object.id, "link7", touch_links);
        RCLCPP_INFO(logger, "Object attached to the end effector!");
    }
    else
    {
        RCLCPP_ERROR(logger, "Planing failed!");
    }
    RCLCPP_INFO(logger, "Planing succeeded!");

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}