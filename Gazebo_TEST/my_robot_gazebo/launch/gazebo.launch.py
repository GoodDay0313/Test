from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,  PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.substitutions import TextSubstitution

def generate_launch_description():
    # Declare arguments
    robot_name = LaunchConfiguration('robot_name', default='my_robot')
    world_name = LaunchConfiguration('world_name', default='storagy.world')

    # Declare launch arguments
    declare_robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='my_robot',
        description='Name of the robot'
    )
    declare_world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='storagy.world',
        description='Name of the Gazebo world file'
    )

    # Set environment variable for Gazebo model path
    gazebo_model_path = [
        '/usr/share/gazebo-11/models/:',
        PathJoinSubstitution([FindPackageShare('my_robot_description'), '..']), ':',
        PathJoinSubstitution([FindPackageShare('my_robot_gazebo'), 'models'])
    ]
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )

    # Include gzserver launch
    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={
            'verbose': 'true',
            'physics': 'ode',
            'lockstep': 'true',
            'world': PathJoinSubstitution([FindPackageShare('my_robot_gazebo'), 'worlds', world_name])
        }.items()
    )

    # Include gzclient launch
    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])
        ])
    )

    # Include robot description upload launch
    upload_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('my_robot_description'), 'launch', 'upload.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'True'
        }.items()
    )

    # Spawn robot entity in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-topic', 'robot_description',
            '-timeout', '20.0',
            '-x', '0.0',
            '-y', '0.0',
            '-package_to_model'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # Load controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        cwd='/home',
        output='screen'
    )

    load_diff_cont = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_cont'],
        cwd='/home',
        output='screen'
    )

    # Create and return the launch description
    return LaunchDescription([
        declare_robot_name_arg,
        declare_world_name_arg,
        set_gazebo_model_path,
        gzserver_launch,
        gzclient_launch,
        upload_launch,
        spawn_robot,
        load_joint_state_broadcaster,
        load_diff_cont
    ])