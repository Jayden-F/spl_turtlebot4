import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument(
            "namespace", default_value="", description="Robot namespace"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use sim time",
        ),
    ]

    remappings = [
        # ("/tf", "tf"),
        # ("/tf_static", "tf_static"),
        # ("/scan", "scan"),
        # ("/map", "map"),
    ]

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2_yaml = os.path.join(
        get_package_share_directory("path_planner_server"), "config", "nav2_params.yaml"
    )

    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = RewrittenYaml(
        source_file=nav2_yaml,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        namespace=namespace,
        name="planner_server",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    controller = Node(
        package="nav2_controller",
        namespace=namespace,
        executable="controller_server",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        namespace=namespace,
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    behaviors = Node(
        package="nav2_behaviors",
        namespace=namespace,
        executable="behavior_server",
        name="behavior_server",
        parameters=[configured_params],
        output="screen",
        remappings=remappings,
    )

    waypoint_follower = Node(
        namespace=namespace,
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_params],
        remappings=remappings,
    )

    lifecycle = Node(
        package="nav2_lifecycle_manager",
        namespace=namespace,
        executable="lifecycle_manager",
        name="lifecycle_manager_pathplanner",
        output="screen",
        remappings=remappings,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 0.0},
            {
                "node_names": [
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                    "waypoint_follower",
                ]
            },
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(planner)
    ld.add_action(controller)
    ld.add_action(bt_navigator)
    ld.add_action(behaviors)
    ld.add_action(waypoint_follower)
    ld.add_action(lifecycle)
    return ld
