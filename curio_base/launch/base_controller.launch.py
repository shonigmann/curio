import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory



''' 
Launch the base controller node

Note:
    This launcher should mimic base_controller.launch for ROS1

Parameters
    classifier_filename : str
        The full filepath for the `scikit-learn` classifier model.
    classifier_window : int
        The size of the classifier window, this sets the number of
        entries in the servo history used to train the classifier.
        The classifier and regressor models must correspond to this
        setting. (default 10)
    regressor_filename : str
        The full filepath for the `scikit-learn` regressor model.
    model : str
        The full filepath to the URDF robot description
    port : str
        The device name for the serial port
'''



def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='classifier_filename',
            default_value=get_package_share_directory(
                'curio_base') + '/data/lx16a_tree_classifier.joblib'
        ),
        launch.actions.DeclareLaunchArgument(
            name='classifier_window',
            default_value='10'
        ),
        launch.actions.DeclareLaunchArgument(
            name='regressor_filename',
            default_value=get_package_share_directory(
                'curio_base') + '/data/lx16a_tree_regressor.joblib'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=get_package_share_directory(
                'curio_description') + '/urdf/curio.urdf.xacro'
        ),
        # Controller for the mobile base
        launch_ros.actions.Node(
            package='curio_base',
            executable='curio_base_controller',
            name='curio_base_controller',
            output='screen',
            parameters=[
                {
                    'classifier_filename': launch.substitutions.LaunchConfiguration('classifier_filename')
                },
                {
                    'classifier_window': launch.substitutions.LaunchConfiguration('classifier_window')
                },
                {
                    'regressor_filename': launch.substitutions.LaunchConfiguration('regressor_filename')
                },
                get_package_share_directory(
                    'curio_base') + '/config/base_controller.yaml'
            ]
        ),
        launch_ros.actions.Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[
                {
                    'use_gui': 'false'
                },
                {
                    'publish_default_positions': 'true'
                },
                {
                    'publish_default_velocities': 'true'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'curio_description'), 'launch/description.launch.py')
            ),
            launch_arguments={
                'model': launch.substitutions.LaunchConfiguration('model')
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
