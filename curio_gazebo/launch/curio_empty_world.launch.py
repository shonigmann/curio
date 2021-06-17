import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, \
    IncludeLaunchDescription, TimerAction, GroupAction, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

# from launch_ros.descriptions import ParameterValue  # NOT AVAILABLE IN FOXY, BUT SEEMS TO BE THE PREFERRED WAY TO
# AVOID ISSUES WITH YAML PARSING OF XACRO COMMAND OUTPUT (https://github.com/ros2/launch_ros/issues/214)


def generate_launch_description():

    ld = LaunchDescription()

    # Get relevant system paths
    curio_gazebo_dir = get_package_share_directory('curio_gazebo')
    curio_description_dir = get_package_share_directory('curio_description')
    realsense_description_dir = get_package_share_directory('realsense2_description')
    world_file = os.path.join(curio_gazebo_dir, 'worlds', 'empty.world')

    # GENERAL SETTINGS:
    ld.add_action(DeclareLaunchArgument('log_level', default_value='WARN', description='Logging level'))

    # GAZEBO ARGUMENTS:
    ld.add_action(DeclareLaunchArgument('gui', default_value='True', description='Whether or not to run Gazebo client'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True', description=''))
    ld.add_action(DeclareLaunchArgument('use_robot_state_pub', default_value='True', description=''))
    ld.add_action(DeclareLaunchArgument('use_simulator', default_value='True',
                                        description='Whether to start the simulator'))
    ld.add_action(DeclareLaunchArgument('world', default_value=world_file,
                                        description='Full path to world model file to load'))
    ld.add_action(DeclareLaunchArgument('paused', default_value='True',
                                        description='Whether or not to start Gazebo server in paused state'))
    
    # add Curio model directories to gazebo model path
    if os.environ.get('GAZEBO_MODEL_PATH') is None:
        os.environ['GAZEBO_MODEL_PATH'] = ''
    ld.add_action(SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
        EnvironmentVariable('GAZEBO_MODEL_PATH'),
        os.path.join(os.pathsep, curio_gazebo_dir, 'models'),
        os.path.join(os.pathsep, curio_description_dir, 'models'),
        os.path.join(os.pathsep, realsense_description_dir, 'models')
    ]))
    
    # GZ SPAWNER ARGUMENTS
    ld.add_action(DeclareLaunchArgument('xacro_path',
                                        default_value=os.path.join(curio_description_dir,
                                            'urdf', 'curio.urdf.xacro'),
                                        description='Path to robot xacro or urdf'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='0.0', description='Initial x position of the robot'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial y position of the robot'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.22', description='Initial z position of the robot'))
    ld.add_action(DeclareLaunchArgument('Y_pose', default_value='0.0', description='Initial Yaw position of the robot'))
    ld.add_action(DeclareLaunchArgument('use_namespace', default_value='False',
                                        description='Whether or not to apply a namespace to the navigation stack'))
    ld.add_action(DeclareLaunchArgument('namespace', default_value='',
                                        description='Top-level navigation stack namespace'))
    ld.add_action(DeclareLaunchArgument('xacro_args', default_value='',
                                        description='Xacro argument inputs used to customize robot generation'))

    # ADVANCED FUNCTIONALITY
    # ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True', description=''))
    # ld.add_action(DeclareLaunchArgument('run_slam', default_value='True', description=''))

    # LAUNCH GAZEBO
    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzserver.launch.py']),
        launch_arguments=[('verbose', 'True'),
                          ('pause', 'True'),
                          ('world', LaunchConfiguration('world')),
                          ('physics', 'ode')]
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzclient.launch.py']),
        launch_arguments=[('verbose', 'True')],
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # REMAP NAMESPACES (NEEDED FOR MULTI-AGENT SIMS
    # (Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # (orduno) Substitute with `PushNodeRemapping` https://github.com/ros2/launch_ros/issues/56)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # START ROBOT STATE PUBLISHER

    # SPAWN ROBOT
    spawn_model = Node(
        package='ros2_xacro_gazebo_spawner',
        executable='ros2_xacro_gazebo_spawner',
        output='screen',
        arguments=[
            '--robot_name', 'curio',
            '--robot_namespace', LaunchConfiguration('namespace'),
            '--urdf', LaunchConfiguration('xacro_path'),
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            '-Y', LaunchConfiguration('Y_pose'),
            # '-p', gz_controller_yaml]),
            '--log-level', LaunchConfiguration('log_level'),
            '--xacro_args', LaunchConfiguration('xacro_args')
        ]
    )
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(LaunchConfiguration('use_robot_state_pub')),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        # namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': Command(['xacro ',
                                           LaunchConfiguration('xacro_path'), LaunchConfiguration('xacro_args')])}
        ],
        remappings=remappings,
        arguments=['--log-level', LaunchConfiguration('log_level')]
    )

    # START CONTROLLERS
    # load_joint_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
    #     output='screen'
    # )
    # 
    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller', 'joint_trajectory_controller'],
    #     output='screen'
    # )
    # 
    # load_effort_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_start_controller', 'effort_controllers'],
    #     output='screen'
    # )
    
    # UNPAUSE SIMULATION, AFTER ROBOT HAS SUCCESSFULLY SPAWNED (AVOIDS NAV2 TIME ISSUES RE: TIMESTAMP EXTRAPOLATION)
    unpause_sim = ExecuteProcess(cmd=['ros2', 'service', 'call', '/unpause_physics', 'std_srvs/srv/Empty'],
                                 output='screen')

    actions = [
        # TODO: READD UNPAUSE
        # RegisterEventHandler(event_handler=OnProcessExit(
        #     target_action=spawn_model,
        #     on_exit=[unpause_sim]
        # )),
        # RegisterEventHandler(event_handler=OnProcessExit(
        #     target_action=load_joint_state_controller,
        #     on_exit=[load_joint_trajectory_controller, load_effort_controller]
        # )),
        gz_server,
        gz_client,
        start_robot_state_publisher_cmd,
        spawn_model,
    ]

    for action in actions:
        ld.add_action(action)

    return ld
