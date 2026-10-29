#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Vector3.h>

class CollisionManager 
{
public:
    CollisionManager(rclcpp::Node::SharedPtr node);

    void addCube(const std::string& cube_id, const geometry_msgs::msg::PoseStamped& cube_pose, double size = 0.05);
    void addTable(const std::string& table_id, const geometry_msgs::msg::PoseStamped& table_pose, tf2::Vector3 size);
    void removeObject(const std::string& object_id);
    void removeAllObjects();

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::string main_frame_id_ = "odom";
    rclcpp::Logger logger_;
};