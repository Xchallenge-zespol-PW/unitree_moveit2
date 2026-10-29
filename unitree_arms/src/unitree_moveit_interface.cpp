#include "unitree_moveit2/unitree_moveit_interface.hpp"

UnitreeMoveitInterface::UnitreeMoveitInterface(rclcpp::Node::SharedPtr node) 
    : node_(node),
      arm_controller_(node, "left_arm", "right_arm"),
      collision_manager_(node),
      tf_buffer_(node->get_clock()),
      tf_listener_(tf_buffer_)
{
}

std::optional<geometry_msgs::msg::PoseStamped> UnitreeMoveitInterface::getTransformByName(const std::string& name)
{
    try
    {
        // Poprawione zapytanie TF2 dla ROS 2 Humble z timeoutem jako tf2::duration
        geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_.lookupTransform(
            "odom", name, tf2::TimePointZero, tf2::durationFromSec(0.5)
        );

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = transformStamped.header.stamp;
        pose.header.frame_id = "odom";
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = transformStamped.transform.translation.z;
        pose.pose.orientation = transformStamped.transform.rotation;

        return pose; 
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(node_->get_logger(), "Nie udało się pobrać TF dla ramki '%s': %s", name.c_str(), ex.what());
        return std::nullopt; 
    }
}

void UnitreeMoveitInterface::init()
{
    for(int i = 1; i <= 4; i++)
    {
        std::string new_cube_name = "cube_" + std::to_string(i);
        auto cube_opt = getTransformByName(new_cube_name);
        
        if(cube_opt.has_value())
        {
            // Przekazujemy cały PoseStamped, bo collision_manager teraz tego oczekuje
            collision_manager_.addCube(new_cube_name, cube_opt.value(), 0.05);
            cubes_pos_[new_cube_name] = cube_opt.value();
        }
    }
    
    auto table_opt = getTransformByName("table");
    if(table_opt.has_value())
    {
        collision_manager_.addTable("table", table_opt.value(), tf2::Vector3{1.8, 0.8, 0.72});
    }
}

bool UnitreeMoveitInterface::moveToCube(const std::string& cube_name, const std::string& arm_name)
{
    if (cubes_pos_.find(cube_name) == cubes_pos_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Nie znaleziono kostki o nazwie: %s", cube_name.c_str());
        return false;
    }

    geometry_msgs::msg::PoseStamped cube_pose = cubes_pos_[cube_name];
    return arm_controller_.moveSingleArmWithCollisionAvoidance(
        arm_name,
        cube_pose,
        tf2::Vector3{0, 0, 0});
}

bool UnitreeMoveitInterface::moveWithCube(const std::string& cube_name, const std::string& arm_name)
{
    if (cubes_pos_.find(cube_name) == cubes_pos_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "Nie znaleziono kostki o nazwie: %s", cube_name.c_str());
        return false;
    }

    geometry_msgs::msg::PoseStamped cube_pose = cubes_pos_[cube_name];
    collision_manager_.removeObject(cube_name);
    
    bool success = arm_controller_.pushCubeWithPilz(
        arm_name,
        cube_pose,
        tf2::Vector3{0.1, 0, 0}); 

    auto updated_pose_opt = getTransformByName(cube_name);
    if(updated_pose_opt.has_value())
    {
        cubes_pos_[cube_name] = updated_pose_opt.value();
        collision_manager_.addCube(cube_name, updated_pose_opt.value(), 0.05);
    }

    return success;
}

bool UnitreeMoveitInterface::moveToStart()
{
    return true;
}