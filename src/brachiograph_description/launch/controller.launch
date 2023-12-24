from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
      brachiograph_description_share_path = get_package_share_directory('brachiograph_description')

      return LaunchDescription([
            SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),
            Node(
                  package='controller_manager',
                  executable='ros2_control_node',
                  parameters=[os.path.join(brachiograph_description_share_path, 'config', 'controller.yaml')],
                  output={
                        'stdout': 'screen',
                        'stderr': 'screen',
                  },
            ),
            Node(
                  package='robot_state_publisher',
                  executable='robot_state_publisher',
                  output='screen',
                  parameters=[os.path.join(brachiograph_description_share_path, 'urdf', 'brachiograph.xacro')],
                  remappings=[('/joint_states', '/brachiograph/joint_states')]
            ),
      ])
