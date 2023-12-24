from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import pathlib

def generate_launch_description():
    # Get the directory of the current package
    pkg_dir = pathlib.Path(__file__).parent.absolute()

    # Declare launch arguments
    launch_controller_arg = DeclareLaunchArgument('launch_controller', default_value='True')
    ip_address_arg = DeclareLaunchArgument('ip_address', default_value='172.0.0.1')
    port_arg = DeclareLaunchArgument('port', default_value='8080')

    # Include gazebo launch file
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'gazebo.launch.py'))
    )

    # Include controller launch file
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'controller.launch.py'))
    )

    # Run joint controller program
    joint_controller_node = Node(
        package='brachiograph_simulation',
        executable='joint_controller',
        output='screen',
        condition=LaunchConfiguration('launch_controller'),
        parameters=[
            {'ip_address': LaunchConfiguration('ip_address')},
            {'port': LaunchConfiguration('port')}
        ]
    )

    return LaunchDescription([
        launch_controller_arg,
        ip_address_arg,
        port_arg,
        gazebo_launch,
        controller_launch,
        joint_controller_node
    ])
