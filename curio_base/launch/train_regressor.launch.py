import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

'''
Launch the LX-16A regressor training node

Note:
    This launcher should mimic train_regressor.launch for ROS1

Parameters
    labeldata_filename : str
        The name of the zipped labelled data csv file
    dataset_filename : str
        The name of the zipped dataset csv file
    regressor_filename : str
        The name of the output model file (python pickled format)
'''

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='labeldata_filename',
            default_value=get_package_share_directory(
                'curio_base') + '/data/lx16a_labelled_data.zip'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dataset_filename',
            default_value=get_package_share_directory(
                'curio_base') + '/data/lx16a_dataset.zip'
        ),
        launch.actions.DeclareLaunchArgument(
            name='regressor_filename',
            default_value=get_package_share_directory(
                'curio_base') + '/data/lx16a_tree_regressor.joblib'
        ),
        launch_ros.actions.Node(
            package='curio_base',
            executable='lx16a_train_regressor.py',
            name='lx16a_train_regressor',
            output='screen',
            parameters=[
                {
                    'labeldata_filename': launch.substitutions.LaunchConfiguration('labeldata_filename')
                },
                {
                    'dataset_filename': launch.substitutions.LaunchConfiguration('dataset_filename')
                },
                {
                    'regressor_filename': launch.substitutions.LaunchConfiguration('regressor_filename')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
