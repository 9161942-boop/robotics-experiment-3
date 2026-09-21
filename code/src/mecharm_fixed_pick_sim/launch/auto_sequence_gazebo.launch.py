from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("mecharm_fixed_pick_sim"))
    sorting_launch = package_share / "launch" / "sorting_gazebo.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("contact_grasp", default_value="false"),
            DeclareLaunchArgument("vacuum_grasp", default_value="false"),
            DeclareLaunchArgument("animation_file", default_value="manual_animation.json"),
            DeclareLaunchArgument("start_index", default_value="0"),
            DeclareLaunchArgument("repeat_count", default_value="1"),
            DeclareLaunchArgument("settle_extra_sec", default_value="0.25"),
            DeclareLaunchArgument("startup_delay_sec", default_value="20.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(sorting_launch)),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "execute_motion": "false",
                    "contact_grasp": LaunchConfiguration("contact_grasp"),
                    "vacuum_grasp": LaunchConfiguration("vacuum_grasp"),
                }.items(),
            ),
            TimerAction(
                period=LaunchConfiguration("startup_delay_sec"),
                actions=[
                    Node(
                        package="mecharm_fixed_pick_sim",
                        executable="auto_sequence_player",
                        name="mecharm_auto_sequence_player",
                        output="screen",
                        parameters=[
                            {"animation_file": LaunchConfiguration("animation_file")},
                            {
                                "start_index": ParameterValue(
                                    LaunchConfiguration("start_index"), value_type=int
                                )
                            },
                            {
                                "repeat_count": ParameterValue(
                                    LaunchConfiguration("repeat_count"), value_type=int
                                )
                            },
                            {
                                "settle_extra_sec": ParameterValue(
                                    LaunchConfiguration("settle_extra_sec"),
                                    value_type=float,
                                )
                            },
                        ],
                    )
                ],
            ),
        ]
    )
