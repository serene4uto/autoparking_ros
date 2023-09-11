from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autoparking_simdata',
            executable='simdata_cam',
            name='simdata_cam',
        ),
        Node(
            package='autoparking_simdata',
            executable='simdata_vehicle',
            name='simdata_vehicle',
        ),
    ])