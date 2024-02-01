import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
from launch.conditions import IfCondition


def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument(
            "namespace", default_value="", description="Robot namespace"
        ),
        DeclareLaunchArgument("rviz", default_value="True", description="start rviz"),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use sim time",
        ),
        DeclareLaunchArgument("map", default_value="", description="map files"),
        DeclareLaunchArgument(
            "params_file", default_value="", description="nav2 params file"
        ),
    ]

    params_file = LaunchConfiguration("params_file")
    map_yaml_file = LaunchConfiguration("map")
    rviz_condition = LaunchConfiguration("rviz")

    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    rviz_node = Node(
        condition=IfCondition(rviz_condition),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            ["-d" + os.path.join(nav2_bringup_share, "rviz", "nav2_default_view.rviz")]
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "bringup_launch.py"),
        ),
        launch_arguments={
            "map": map_yaml_file,
            "params_file": params_file,
        }.items(),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(rviz_node)
    ld.add_action(nav2_launch)

    return ld
