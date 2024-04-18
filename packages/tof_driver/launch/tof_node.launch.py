from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

def generate_launch_description():
    veh = DeclareLaunchArgument('veh', default_value='duckiebot')
    name = DeclareLaunchArgument('name', default_value='front_center')

    pkg_name = 'tof_driver'
    node_name = 'tof_node'

    node = Node(
        package=pkg_name,
        namespace=LaunchConfiguration('veh'),
        executable=node_name + '.py',
        output='screen',
        parameters=[
            get_package_share_directory(pkg_name) + '/config/' + node_name + '/front_center.yaml',
            {'veh': LaunchConfiguration('veh')}
        ],
        remappings=[
            (f"front_center_tof_driver_node/fragments", "display_driver_node/fragments")
        ],
        name=f"front_center_tof_driver_node"
    )

    return LaunchDescription([
        veh,
        name,
        node
    ])