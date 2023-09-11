from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the path to the package's 'share' directory
    # package_share_directory = get_package_share_directory('autoparking_simdata')

    return LaunchDescription([
        # DeclareLaunchArgument('simdata_cam_cfg', default_value=os.path.join(package_share_directory,'simdata_cam_cfg.yaml'), 
        #                       description='Path to the parameter file'),

        # Log a message to indicate that we are loading the parameter file
        # LogInfo(
        #     condition=IfCondition(LaunchConfiguration('simdata_cam_cfg')),
        #     msg=[f'Loading parameters from {LaunchConfiguration("simdata_cam_cfg")}']
        # ),

        Node(
            package='autoparking_simdata',
            executable='simdata_cam',
            name='simdata_cam',
            parameters=[
                # LaunchConfiguration('simdata_cam_cfg')
                {'data_path': '/workspaces/Project_APS_new'}
            ]
        ),
        Node(
            package='autoparking_simdata',
            executable='simdata_vehicle',
            name='simdata_vehicle',
        ),
    ])