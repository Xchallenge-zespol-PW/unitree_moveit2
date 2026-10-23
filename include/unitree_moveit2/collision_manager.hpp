#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class CollisionManager
{
public:
    explicit CollisionManager(rclcpp::Node::SharedPtr node);

    // Dodanie kostki (sześcianu) do sceny kolizji
    void addCube(const std::string& cube_id, const geometry_msgs::msg::Pose& cube_pose, double size = 0.05);

    // Dodanie stołu (blatu) z określoną pozycją i orientacją oraz wymiarami
    void addTable(const std::string& table_id, 
                  const geometry_msgs::msg::Pose& table_pose, 
                  double size_x = 1.8, double size_y = 0.8, double size_z = 0.72);

    // Usunięcie konkretnego obiektu ze sceny (np. kostki przed pchaniem)
    void removeObject(const std::string& object_id);

    // Usunięcie wszystkich obiektów ze sceny kolizji
    void removeAllObjects();

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    rclcpp::Logger logger_;
};