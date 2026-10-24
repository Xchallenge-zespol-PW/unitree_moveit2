#include <rclcpp/rclcpp.hpp>
#include "unitree_moveit2/collision_manager.hpp"
#include "unitree_moveit2/arm_controller.hpp"



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("unitree_test_node");

    // Inicjalizacja obu klas na tym samym wskaźniku węzła
    CollisionManager collision_mgr(node);
    ArmController arm_ctrl(node, "left_panda_arm", "right_panda_arm");

    // 1. Dodanie stołu do sceny kolizji
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.5;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.30; 
    table_pose.orientation.w = 0.707;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = 0.707;
    collision_mgr.addTable("table", table_pose);

    // 2. Przykładowy ruch lewego ramienia z omijaniem kolizji
    geometry_msgs::msg::Pose target;
    target.position.x = 0.54;
    target.position.y = 0.0;
    target.position.z = 0.51;

    collision_mgr.addCube("cube_0", target, 0.05);

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    if(arm_ctrl.moveSingleArmWithCollisionAvoidance("left_panda_arm", target))
        arm_ctrl.pushObjectWithServo("left_hand", 0.1, 5.0);

    rclcpp::shutdown();
    return 0;
}