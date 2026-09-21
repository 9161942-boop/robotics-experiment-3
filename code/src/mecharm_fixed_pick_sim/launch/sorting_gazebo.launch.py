from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("mecharm_fixed_pick_sim"))
    official_launch = package_share / "launch" / "official_assembled_gazebo.launch.py"
    sorting_world = package_share / "worlds" / "sorting_desktop.world"
    sorting_config = package_share / "config" / "official_fixed_pick.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("execute_motion", default_value="false"),
        DeclareLaunchArgument("contact_grasp", default_value="false"),
        DeclareLaunchArgument("vacuum_grasp", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(official_launch)),
            launch_arguments={
                "world": str(sorting_world),
                "config_file": str(sorting_config),
                "run_task": "false",
                "gui": LaunchConfiguration("gui"),
                "contact_grasp": LaunchConfiguration("contact_grasp"),
                "vacuum_grasp": LaunchConfiguration("vacuum_grasp"),
            }.items(),
        ),
        Node(
            package="mecharm_fixed_pick_sim",
            executable="sorting_scene_monitor",
            name="sorting_scene_monitor",
            output="screen",
        ),
        Node(
            package="mecharm_fixed_pick_sim",
            executable="sorting_task",
            name="sorting_task",
            output="screen",
            parameters=[{"execute_motion": LaunchConfiguration("execute_motion")}],
        ),
        Node(
            package="mecharm_fixed_pick_sim",
            executable="sorting_executor",
            name="sorting_executor",
            output="screen",
            parameters=[{"execute_motion": LaunchConfiguration("execute_motion")}],
        ),
    ])
