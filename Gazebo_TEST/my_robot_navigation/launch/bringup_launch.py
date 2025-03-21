# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
def generate_launch_description():
    # Get the launch directory
    # TODO: 수정
    MOBILE_MODEL = "my_robot_navigation"
    Gazebo_MODEL = 'my_robot_gazebo'

    bringup_dir = get_package_share_directory(MOBILE_MODEL)
    launch_dir = os.path.join(bringup_dir, 'launch')
    Gazebo_Dir=  os.path.join(get_package_share_directory(Gazebo_MODEL), 'launch')
    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_z = LaunchConfiguration('initial_pose_z')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
                get_package_share_directory('my_robot_navigation'), 'maps', 'realMap.yaml'
            ),
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    declare_initial_x = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='X coordinate of the initial robot pose in the map frame"')

    declare_initial_y =DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Y coordinate of the initial robot pose in the map frame"')

    declare_initial_z =DeclareLaunchArgument(
        'initial_pose_z',
        default_value='0.0',
        description='Z coordinate of the initial robot pose in the map frame"')

    declare_initial_yaw =DeclareLaunchArgument(
        'initial_pose_yaw',
        default_value='0.0',
        description='Yaw of the initial robot pose in the map frame"')    

    declare_default_nav_to_pose_bt_xml =DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=os.path.join(bringup_dir, 'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Full path to the xml file used by nav to pose')    

    declare_default_nav_through_poses_bt_xml =DeclareLaunchArgument(
        'default_nav_through_poses_bt_xml',
        default_value=os.path.join(bringup_dir, 'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml'),
        description='Full path to the xml file used by nav through pose')     
    
    
    default_rviz_config_path = DeclareLaunchArgument(
        'default_rviz_config_path',
        default_value=os.path.join(get_package_share_directory("my_robot_navigation"), 'rviz', 'nav2_view.rviz')
    )
    # Set Gazebo model path
   
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('default_rviz_config_path')]
    )
    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'slam_launch.py')),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'localization.launch.py')),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'initial_pose_x': initial_pose_x,
                              'initial_pose_y': initial_pose_y,
                              'initial_pose_z': initial_pose_z,
                              'initial_pose_yaw': initial_pose_yaw,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container'}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container',
                              'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
                              'default_nav_through_poses_bt_xml' :default_nav_through_poses_bt_xml}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(Gazebo_Dir, 'gazebo.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'robot_name': 'my_robot',  # my_robot_launch.py에서 사용되는 인자
                              'world_name': 'storagy.world'}.items()),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_default_nav_to_pose_bt_xml)
    ld.add_action(declare_default_nav_through_poses_bt_xml)
    ld.add_action(declare_initial_x)
    ld.add_action(declare_initial_y)
    ld.add_action(declare_initial_z)
    ld.add_action(declare_initial_yaw)
    ld.add_action(default_rviz_config_path)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)
    
    ld.add_action(rviz_node)
    return ld