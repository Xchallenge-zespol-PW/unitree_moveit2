#include "unitree_moveit2/collision_manager.hpp"
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

CollisionManager::CollisionManager(rclcpp::Node::SharedPtr node)
    : node_(node), logger_(rclcpp::get_logger("CollisionManager"))
{
}

void CollisionManager::addCube(const std::string& cube_id, const geometry_msgs::msg::Pose& cube_pose, double size)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link"; // Upewnij się, że to ten sam frame bazowy
    collision_object.id = cube_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {size, size, size};

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cube_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_.applyCollisionObject(collision_object);
    RCLCPP_INFO(logger_, "Dodano kostkę: %s do sceny kolizji.", cube_id.c_str());
}

void CollisionManager::addTable(const std::string& table_id, 
                              const geometry_msgs::msg::Pose& table_pose, 
                              double size_x, double size_y, double size_z)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = table_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {size_x, size_y, size_z};

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(table_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface_.applyCollisionObject(collision_object);
    RCLCPP_INFO(logger_, "Dodano stół: %s do sceny kolizji.", table_id.c_str());
}

void CollisionManager::removeObject(const std::string& object_id)
{
    planning_scene_interface_.removeCollisionObjects({object_id});
    RCLCPP_INFO(logger_, "Usunięto obiekt: %s ze sceny kolizji.", object_id.c_str());
}

void CollisionManager::removeAllObjects()
{
    auto object_ids = planning_scene_interface_.getKnownObjectNames();
    if (!object_ids.empty())
    {
        planning_scene_interface_.removeCollisionObjects(object_ids);
        RCLCPP_INFO(logger_, "Usunięto wszystkie obiekty ze sceny kolizji.");
    }
}