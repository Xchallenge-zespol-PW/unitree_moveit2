#include "unitree_moveit2/arm_controller.hpp"

ArmController::ArmController(rclcpp::Node::SharedPtr node,
                             const std::string& left_group_name,
                             const std::string& right_group_name)
    : node_(node),
      left_arm_(node, left_group_name),
      right_arm_(node, right_group_name),
      logger_(rclcpp::get_logger("ArmController"))
{
    left_arm_.setPlanningTime(5.0);
    right_arm_.setPlanningTime(5.0);


    servo_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);
}

bool ArmController::moveToStart(const geometry_msgs::msg::Pose& target_left,
                                const geometry_msgs::msg::Pose& target_right)
{
    RCLCPP_INFO(logger_, "Ruch ramion do pozycji startowej...");

    left_arm_.setPoseTarget(target_left);
    right_arm_.setPoseTarget(target_right);

    moveit::planning_interface::MoveGroupInterface::Plan plan_left, plan_right;
    
    if (left_arm_.plan(plan_left) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Błąd planowania dla lewego ramienia!");
        return false;
    }
    left_arm_.execute(plan_left);

    if (right_arm_.plan(plan_right) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Błąd planowania dla prawego ramienia!");
        return false;
    }
    right_arm_.execute(plan_right);

    return true;
}

void ArmController::pushObjectWithServo(const std::string& ee_frame_name,
                                      double linear_speed_x, 
                                      double duration_sec)
{
    RCLCPP_INFO(logger_, "Rozpoczęcie pchania kostki za pomocą MoveIt Servo...");

    rclcpp::Rate rate(100); 
    auto start_time = node_->now();

    while (rclcpp::ok() && (node_->now() - start_time).seconds() < duration_sec)
    {
        geometry_msgs::msg::TwistStamped twist_msg;
        twist_msg.header.stamp = node_->now();
        twist_msg.header.frame_id = ee_frame_name; 


        twist_msg.twist.linear.x = linear_speed_x;
        twist_msg.twist.linear.y = 0.0;
        twist_msg.twist.linear.z = 0.0;
        twist_msg.twist.angular.x = 0.0;
        twist_msg.twist.angular.y = 0.0;
        twist_msg.twist.angular.z = 0.0;

        servo_publisher_->publish(twist_msg);
        rate.sleep();
    }

    geometry_msgs::msg::TwistStamped stop_msg;
    stop_msg.header.stamp = node_->now();
    stop_msg.header.frame_id = ee_frame_name;
    servo_publisher_->publish(stop_msg);
    
    RCLCPP_INFO(logger_, "Zakończono pchanie kostki.");
}