#include "unitree_moveit2/unitree_moveit_interface.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("unitree_test_node");

    RCLCPP_INFO(node->get_logger(), "Uruchamianie interfejsu testowego MoveIt...");
    UnitreeMoveitInterface robot_interface(node);


    robot_interface.init();


    rclcpp::sleep_for(std::chrono::seconds(2));


    RCLCPP_INFO(node->get_logger(), "Test: Ruch do cube_1...");
    robot_interface.moveToCube("cube_1", "left_arm");


    RCLCPP_INFO(node->get_logger(), "Test: Pchanie cube_1 za pomocą Pilza...");
    robot_interface.moveWithCube("cube_1", "left_arm");

    rclcpp::shutdown();
    return 0;
}