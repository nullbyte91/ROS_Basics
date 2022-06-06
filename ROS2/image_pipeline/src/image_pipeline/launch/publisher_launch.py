import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
    get_package_share_directory('image_pipeline'),
    'config',
    'parameters.yaml'
    )
    node1 = Node(
            package='image_pipeline',
            executable='img_publisher',
            output='screen',
            parameters = [config]
    )
    node2 = Node(
            package='image_pipeline',
            executable='img_subscriber',
            output='screen',
            parameters = [config]
    )
    ld.add_action(node1)
    ld.add_action(node2)
    return ld