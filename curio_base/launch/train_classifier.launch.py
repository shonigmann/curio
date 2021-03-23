import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

'''
Launch the LX-16A classifier training node

Note:
    This launcher should mimic base_controller.launch for ROS1

Parameters
    check_accuracy_score : bool
        Set to true to run the accuracy score
    check_cross_validation_score : bool
        Set to true to run the cross validation score (takes time)
    dataset_filename : str
        The name of the zipped dataset csv file
    classifier_filename : str
        The name of the output model file (python pickled format)
'''

def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='check_accuracy_score',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='check_cross_validation_score',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='dataset_filename',
            default_value=get_package_share_directory(
                'curio_base') + '/data/lx16a_dataset.zip'
        ),
        launch.actions.DeclareLaunchArgument(
            name='classifier_filename',
            default_value=get_package_share_directory(
                'curio_base') + '/data/lx16a_tree_classifier.joblib'
        ),
        # Run the training script for the classifier used the encoder filter
        launch_ros.actions.Node(
            package='curio_base',
            executable='lx16a_train_classifier.py',
            name='lx16a_train_classifier',
            output='screen',
            parameters=[
                {
                    'check_accuracy_score': launch.substitutions.LaunchConfiguration('check_accuracy_score')
                },
                {
                    'check_cross_validation_score': launch.substitutions.LaunchConfiguration('check_cross_validation_score')
                },
                {
                    'dataset_filename': launch.substitutions.LaunchConfiguration('dataset_filename')
                },
                {
                    'classifier_filename': launch.substitutions.LaunchConfiguration('classifier_filename')
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
