from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():
    use_sim_time = True
    robot_description_path = os.path.join(
        get_package_share_directory('my_robot_description'),
        'storagy_urdf',
        'storagy_robot.urdf.xacro'
    )
    robot_description = Command([
        'xacro ',
        robot_description_path,
        
    ])
       

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node,
    ])