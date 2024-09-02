from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roboticpark_cyberattacks',
            namespace='dos',
            executable='dosattack',
            name='mydos'
        ),
    ])