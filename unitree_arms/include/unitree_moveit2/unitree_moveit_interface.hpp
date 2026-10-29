#pragma once

#include <string>
#include <unordered_map>
#include <optional>
#include <rclcpp/rclcpp.hpp>

#include "unitree_moveit2/collision_manager.hpp"
#include "unitree_moveit2/arm_controller.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class UnitreeMoveitInterface 
{
public:
    UnitreeMoveitInterface(rclcpp::Node::SharedPtr node);
    void init();
    bool moveToCube(const std::string& cube_name, const std::string& arm_name);
    bool moveWithCube(const std::string& cube_name, const std::string& arm_name);
    bool moveToStart();

private:
    std::optional<geometry_msgs::msg::PoseStamped> getTransformByName(const std::string& name);

    rclcpp::Node::SharedPtr node_;
    ArmController arm_controller_;
    CollisionManager collision_manager_;

    // Narzędzia TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Mapa przechowująca pozycje kostek
    std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> cubes_pos_;
};