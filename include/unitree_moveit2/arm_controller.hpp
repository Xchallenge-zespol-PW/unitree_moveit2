// arm_controller.hpp (wersja czysta)
#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

class ArmController 
{
public:
    ArmController(rclcpp::Node::SharedPtr node,
                  const std::string& left_group_name = "left_arm",
                  const std::string& right_group_name = "right_arm");

    bool moveSingleArmWithCollisionAvoidance(const std::string& active_arm_name, const geometry_msgs::msg::Pose& target_pose);
    
    void pushObjectWithServo(const std::string& ee_frame_name,
                             double linear_speed_x, 
                             double duration_sec);

private:
    rclcpp::Node::SharedPtr node_;
    moveit::planning_interface::MoveGroupInterface left_arm_;
    moveit::planning_interface::MoveGroupInterface right_arm_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr servo_publisher_;
    rclcpp::Logger logger_;
};