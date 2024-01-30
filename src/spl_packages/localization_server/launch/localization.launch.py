import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # rviz_config = os.path.join(get_package_share_directory(
    #     'localization_server'), 'config', 'localization_rviz_config.rviz')
    amcl_yaml = os.path.join(
        get_package_share_directory("localization_server"), "config", "amcl_config.yaml"
    )
    map_file = os.path.join(
        get_package_share_directory("localization_server"), "maps", "full_office.yaml"
    )

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

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = RewrittenYaml(
        source_file=amcl_yaml,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    remappings = [
        # ("/tf", "tf"),
        # ("/tf_static", "tf_static"),
        # ("/scan", "scan"),
        # ("/map", "map"),
    ]

    map_server = Node(
        package="nav2_map_server",
        namespace=namespace,
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}, {"yaml_filename": map_file}],
    )

    amcl_server = Node(
        package="nav2_amcl",
        namespace=namespace,
        executable="amcl",
        name="amcl_server",
        parameters=[configured_params, {"tf_options.ignore_time_diff_warning": 1}],
        remappings=remappings,
        output="screen",
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        namespace=namespace,
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 0.0},
            {"node_names": ["map_server", "amcl_server"]},
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(map_server)
    ld.add_action(amcl_server)
    ld.add_action(lifecycle_manager)
    return ld
