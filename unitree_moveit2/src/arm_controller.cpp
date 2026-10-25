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

geometry_msgs::msg::Pose generateStartPose(
    const geometry_msgs::msg::Pose& target_pose, 
    double x_offset, 
    double y_offset = 0.0, 
    double z_offset = 0.0,
    bool flip_down = true)
{
    geometry_msgs::msg::Pose start_pose;

    tf2::Quaternion q_target;
    tf2::fromMsg(target_pose.orientation, q_target);

    if (flip_down)
    {
        tf2::Quaternion q_flip(1.0, 0.0, 0.0, 0.0); 
        
        q_target = q_target * q_flip;
    }

    start_pose.orientation = tf2::toMsg(q_target);

    tf2::Vector3 local_offset(x_offset, y_offset, z_offset);

    tf2::fromMsg(target_pose.orientation, q_target);

    tf2::Vector3 global_offset = tf2::quatRotate(q_target, local_offset);

    start_pose.position.x = target_pose.position.x + global_offset.x();
    start_pose.position.y = target_pose.position.y + global_offset.y();
    start_pose.position.z = target_pose.position.z + global_offset.z();

    return start_pose;
}

bool ArmController::moveSingleArmWithCollisionAvoidance(
     const std::string& active_arm_name,
     const geometry_msgs::msg::Pose& target_pose,
     double offset_x,
     double offset_y,
     double offset_z,
     bool flip_down)
{
    using moveit::planning_interface::MoveGroupInterface;
    
    
    std::string other_arm_name = (active_arm_name == "left_panda_arm") ? "right_panda_arm" : "left_panda_arm";
    
    MoveGroupInterface active_arm(node_, active_arm_name);
    MoveGroupInterface other_arm(node_, other_arm_name);

    active_arm.setStartStateToCurrentState();

    
    geometry_msgs::msg::Pose start_pose = generateStartPose(target_pose, offset_x, offset_y, offset_z, flip_down);

    active_arm.setPoseTarget(start_pose);
    active_arm.setPlanningTime(5.0);
    RCLCPP_INFO(logger_, "Planning frame: %s", active_arm.getPlanningFrame().c_str());
    MoveGroupInterface::Plan plan_active;
    if (active_arm.plan(plan_active) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Nie udało się zaplanować ruchu dla ramienia: %s", active_arm_name.c_str());
        return false;
    }

    // 3. Wykonanie ruchu
    active_arm.execute(plan_active);
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