import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

autopatrol_robot_share_path = get_package_share_directory('autopatrol_robot')
autopatrol_robot_param_path = os.path.join(autopatrol_robot_share_path, 'config', 'patrol_config.yaml')

def generate_launch_description():

    action_patrol_node = Node(
            package='autopatrol_robot',
            executable='patrol_node',
            parameters=[autopatrol_robot_param_path],
    )

    action_speaker = Node(
            package='autopatrol_robot',
            executable='speaker',
    )
    

    return LaunchDescription([
        action_patrol_node,
        action_speaker,
    ])
