from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory 
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboticpark_cyberattacks',
            executable='replyattack_fake_node',
            name='replynodefakenode',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('roboticpark_cyberattacks'), 'config', 'reply.params.fake.node.yaml')]
        ),
    ])