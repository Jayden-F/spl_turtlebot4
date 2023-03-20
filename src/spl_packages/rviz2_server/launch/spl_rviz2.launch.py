import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    ARGUMENTS = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Robot namespace'),

        DeclareLaunchArgument('use_sim_time', default_value='false',
                              choices=['true', 'false'],
                              description='Use sim time')
    ]

    namespace = LaunchConfiguration('namespace')

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static")
    ]

    rviz_config = os.path.join(get_package_share_directory(
        'rviz2_server'), "config", "rviz_config.rviz")

    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')

    description_launch = PathJoinSubstitution(
        [pkg_turtlebot4_description, 'launch', 'robot_description.launch.py']
    )

    # rviz = GroupAction([
    #     PushRosNamespace(namespace),

    rviz = Node(
        package="rviz2",
        namespace=namespace,
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=['-d' + rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=remappings
    )

    # Delay launch of robot description to allow Rviz2 to load first.
    # Prevents visual bugs in the model.
    robot_description = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource([description_launch]),
            launch_arguments=[
                {'model': "lite", 'namespace': namespace}.items()]
        )]
    )

    ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(robot_description)
    ld.add_action(rviz)

    return ld
