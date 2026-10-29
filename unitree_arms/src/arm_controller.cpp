#include "unitree_moveit2/arm_controller.hpp"

ArmController::ArmController(rclcpp::Node::SharedPtr node,
                             const std::string& left_group_name,
                             const std::string& right_group_name)
    : node_(node),
      left_arm_(node, left_group_name),
      right_arm_(node, right_group_name),
      logger_(rclcpp::get_logger("ArmController")),
      left_group_name_(left_group_name),
      right_group_name_(right_group_name)
{
    left_arm_.setPlanningTime(5.0);
    right_arm_.setPlanningTime(5.0);

    servo_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/servo_node/delta_twist_cmds", 10);
}

geometry_msgs::msg::PoseStamped generateStartPose(
    const geometry_msgs::msg::PoseStamped& target_pose, 
    tf2::Vector3 offset)
{
    geometry_msgs::msg::PoseStamped start_pose;
    
    start_pose.header = target_pose.header;
    start_pose.pose.orientation = target_pose.pose.orientation;

    tf2::Quaternion q_target;
    tf2::fromMsg(target_pose.pose.orientation, q_target);

    tf2::Vector3 global_offset = tf2::quatRotate(q_target, offset);

    start_pose.pose.position.x = target_pose.pose.position.x + global_offset.x();
    start_pose.pose.position.y = target_pose.pose.position.y + global_offset.y();
    start_pose.pose.position.z = target_pose.pose.position.z + global_offset.z();

    return start_pose;
}

bool ArmController::moveSingleArmWithCollisionAvoidance(
     const std::string& active_arm_name,
     const geometry_msgs::msg::PoseStamped& target_pose,
     tf2::Vector3 offset)
{
    using moveit::planning_interface::MoveGroupInterface;
    
    std::string other_arm_name = (active_arm_name == left_group_name_) ? right_group_name_ : left_group_name_;
    
    MoveGroupInterface active_arm(node_, active_arm_name);
    MoveGroupInterface other_arm(node_, other_arm_name);

    active_arm.setStartStateToCurrentState();
    
    geometry_msgs::msg::PoseStamped start_pose = generateStartPose(target_pose, offset);

    active_arm.setPoseTarget(start_pose);
    active_arm.setPlanningTime(5.0);

    active_arm.setPlanningPipelineId("ompl");
    active_arm.setPlannerId("RRTConnect");

    RCLCPP_INFO(logger_, "Planning frame (RRTConnect): %s | Target frame: %s", 
                active_arm.getPlanningFrame().c_str(), 
                target_pose.header.frame_id.c_str());
                
    MoveGroupInterface::Plan plan_active;
    if (active_arm.plan(plan_active) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Nie udało się zaplanować ruchu (RRTConnect) dla ramienia: %s", active_arm_name.c_str());
        return false;
    }

    active_arm.execute(plan_active);
    return true;
}

bool ArmController::pushCubeWithPilz(
     const std::string& active_arm_name,
     const geometry_msgs::msg::PoseStamped& target_pose,
     tf2::Vector3 push_offset)
{
    using moveit::planning_interface::MoveGroupInterface;
    
    MoveGroupInterface active_arm(node_, active_arm_name);
    active_arm.setStartStateToCurrentState();
    
    geometry_msgs::msg::PoseStamped final_pose = target_pose;
    
    final_pose.pose.position.x = target_pose.pose.position.x + push_offset.x();
    final_pose.pose.position.y = target_pose.pose.position.y + push_offset.y();
    final_pose.pose.position.z = target_pose.pose.position.z + push_offset.z();

    active_arm.setPoseTarget(final_pose);
    active_arm.setPlanningTime(5.0);

    active_arm.setPlanningPipelineId("pilz_industrial_motion_planner");
    active_arm.setPlannerId("LIN"); 

    RCLCPP_INFO(logger_, "Planning frame (Pilz LIN - pchanie równoległe): %s | Target frame: %s", 
                active_arm.getPlanningFrame().c_str(), 
                target_pose.header.frame_id.c_str());
    
    MoveGroupInterface::Plan plan_active;
    if (active_arm.plan(plan_active) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Nie udało się zaplanować ruchu liniowego (Pilz LIN) dla ramienia: %s", active_arm_name.c_str());
        return false;
    }

    active_arm.execute(plan_active);
    return true;
}