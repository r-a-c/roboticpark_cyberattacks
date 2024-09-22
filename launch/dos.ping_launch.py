from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory 
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboticpark_cyberattacks',
#           namespace='dos',
            executable='dosattack',
            name='dosnode',
            parameters=[os.path.join(get_package_share_directory('roboticpark_cyberattacks'), 'config', 'dos.params.ping.yaml')]
        ),
        Node(
            package='roboticpark_cyberattacks',
#           namespace='dos',
            executable='logcapturer',
            name='logcapturer',
            parameters=[os.path.join(get_package_share_directory('roboticpark_cyberattacks'), 'config', 'logcapturer.params.yaml')]
        ),
    ])