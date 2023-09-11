from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autoparking_core',
            executable='bridge2server',
            name='bridge2server',
        ),
        Node(
            package='autoparking_core',
            executable='pslot_status_detector',
            name='pslot_status_detector',
        ),
    ])