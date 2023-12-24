from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  gazebo_ros_share_path = get_package_share_directory('gazebo_ros')
  brachiograph_description_share_path = get_package_share_directory('brachiograph_description')

  return LaunchDescription([
    ExecuteProcess(
      cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so'],
      output='screen'
    ),
    # Publish the static transforms from the URDF
    Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      output='screen',
      parameters=[os.path.join(brachiograph_description_share_path, 'urdf', 'brachiograph.xacro')]
    ),
    # Call spawn_model service from gazebo_ros to spawn a URDF robot
    Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=['-entity', 'brachiograph', '-file', os.path.join(brachiograph_description_share_path, 'urdf', 'brachiograph.xacro')],
      output='screen'
    )
  ])
