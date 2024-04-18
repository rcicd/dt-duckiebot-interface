from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PathJoinSubstitution
from ament_index_python import get_package_share_directory


def generate_launch_description():
    veh = DeclareLaunchArgument('veh', default_value='duckiebot')
    wheel = DeclareLaunchArgument('wheel', default_value='default')
    node_name = DeclareLaunchArgument('node_name', default_value='wheel_encoder_node')
    param_file_name = DeclareLaunchArgument('param_file_name', default_value='common_wheel.yaml')

    pkg_name = 'wheel_encoder'

    node = Node(
        package=pkg_name,
        namespace=LaunchConfiguration('veh'),
        executable='wheel_encoder_node.py',
        name=[LaunchConfiguration('wheel'), '_wheel_encoder_node'],
        output='screen',
        remappings=[
            ([LaunchConfiguration('wheel'), "_wheel_encoder_node/wheels_cmd_executed"], "wheels_driver_node/wheels_cmd_executed"),
        ],
        parameters=[
            {"veh": LaunchConfiguration('veh')},
            {"name": LaunchConfiguration('wheel')},
            PathJoinSubstitution([
                get_package_share_directory('wheel_encoder') + '/config/wheel_encoder_node',
                LaunchConfiguration('param_file_name')
            ])
        ]
    )

    return LaunchDescription([
        veh,
        wheel,
        node_name,
        param_file_name,
        node
    ])