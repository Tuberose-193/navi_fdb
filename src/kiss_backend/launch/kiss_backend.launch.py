import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command

sys.path.append(os.path.join(get_package_share_directory('kiss_backend'), 'launch'))
def generate_launch_description():
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node,SetParameter
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription 
    node_params = os.path.join(get_package_share_directory('kiss_backend'), 'config', 'kiss_backend.yaml')
    kiss_backend_node = Node(
        package='kiss_backend',
        executable='kiss_backend_node',
        output='screen',
        parameters=[node_params],
        respawn=True,
        respawn_delay=1)


    return LaunchDescription([
        kiss_backend_node
    ])
