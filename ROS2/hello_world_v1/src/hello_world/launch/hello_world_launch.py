import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
    get_package_share_directory('hello_world'),
    'config',
    'parameters.yaml'
    )
    node1 = Node(
            package='hello_world',
            executable='helloworld',
            output='screen',
            parameters = [config]
    )
    ld.add_action(node1)
    return ld