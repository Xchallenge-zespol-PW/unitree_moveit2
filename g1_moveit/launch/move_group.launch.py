from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def build_moveit_config():
    # Jawne wskazanie ścieżki do pakietu konfiguracyjnego G1
    pkg_share = Path(get_package_share_directory("g1_moveit"))
    
    config = (
        MoveItConfigsBuilder(
            robot_name="g1_29dof",
            package_name="g1_moveit",
        )
        .robot_description(file_path=str(pkg_share / "config" / "g1_29dof.urdf")) # lub odpowiednik URDF
        .robot_description_semantic(file_path="config/g1_29dof.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    return config

def _launch_sdk_setup(context):
    use_sim_time_val = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    
    # Budujemy konfigurację MoveIt wewnątrz OpaqueFunction
    moveit_config = build_moveit_config()
    
    # Pobieramy słownik parametrów i wstrzykujemy czas symulacji
    move_group_parameters = moveit_config.to_dict()
    move_group_parameters["use_sim_time"] = use_sim_time_val

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[move_group_parameters],
    )

    return [move_group_node]

def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock from /clock",
            ),
            OpaqueFunction(function=_launch_sdk_setup),
        ]
    )