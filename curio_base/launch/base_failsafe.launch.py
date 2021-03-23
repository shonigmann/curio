import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

'''

Launch the base failsafe node

Note:
    This launcher should mimic base_failsafe.launch for ROS1

Parameters (defined in base_controller.yaml)
    port : str
        The device name for the serial port
    failsafe_control_frequency : str
        The frequency for the failsafe control loop

'''

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='curio_base',
            executable='curio_base_failsafe',
            name='curio_base_failsafe',
            output='screen',
            parameters=[
                get_package_share_directory('curio_base') + '/config/base_controller.yaml'
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
