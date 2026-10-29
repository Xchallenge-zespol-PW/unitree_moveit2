#pragma once

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

class ArmController 
{
public:
    ArmController(rclcpp::Node::SharedPtr node,
                  const std::string& left_group_name = "left_arm",
                  const std::string& right_group_name = "right_arm");

    bool moveSingleArmWithCollisionAvoidance(const std::string& active_arm_name,
                    const geometry_msgs::msg::PoseStamped& target_pose,
                    tf2::Vector3 offset);

    bool pushCubeWithPilz(const std::string& active_arm_name,
                    const geometry_msgs::msg::PoseStamped& target_pose,
                    tf2::Vector3 push_offset);

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface left_arm_;
    moveit::planning_interface::MoveGroupInterface right_arm_;
    
    std::string left_group_name_;
    std::string right_group_name_;
    
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_publisher_;
    rclcpp::Logger logger_;
};