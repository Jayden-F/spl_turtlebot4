import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace


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

    # Packages
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    path_planner_package = get_package_share_directory(
        'path_planner_server')

    cartographer_slam_package = get_package_share_directory(
        'cartographer_slam')

    rviz_package = get_package_share_directory("rviz2_server")

    turtlebot4_navigation_dir = get_package_share_directory(
        "turtlebot4_navigation")

    localization_dir = get_package_share_directory(
        "localization_server")

    # Launch Files
    path_planner_launch = PathJoinSubstitution(
        [path_planner_package, 'launch', 'pathplanner.launch.py'])

    # cartographer_slam_launch = PathJoinSubstitution(
    #     [cartographer_slam_package, 'launch', 'cartographer.launch.py'])

    rviz_launch = PathJoinSubstitution(
        [rviz_package, 'launch', 'spl_rviz2.launch.py'])

    slam_toolbox_launch = PathJoinSubstitution([turtlebot4_navigation_dir, 'launch',
                                       'slam.launch.py'])

    localization_launch = PathJoinSubstitution([localization_dir, "launch", "localization.launch.py"])

    # Actions
    navigation =   IncludeLaunchDescription(
            PythonLaunchDescriptionSource([path_planner_launch]), launch_arguments={"namespace": namespace, "use_sim_time": use_sim_time}.items())

    localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([localization_launch]), launch_arguments={"namespace": namespace, "use_sim_time": use_sim_time}.items())


    mapping = IncludeLaunchDescription(PythonLaunchDescriptionSource([slam_toolbox_launch]), launch_arguments={"namespace": namespace, "use_sim_time": use_sim_time}.items())


    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch]), launch_arguments={})

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(navigation)
    ld.add_action(localization)
    # ld.add_action(mapping)
    ld.add_action(rviz)

    return ld
