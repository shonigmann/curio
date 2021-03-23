import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

''' 
Load the robot description to the ROS2 parameter server

Note:
    This launcher should mimic description.launch for ROS1
''' 

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=get_package_share_directory(
                'curio_description') + '/urdf/curio.urdf.xacro'
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()
