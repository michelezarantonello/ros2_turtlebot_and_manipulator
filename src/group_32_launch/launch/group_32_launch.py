from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    included_launch_file = os.path.join(
        get_package_share_directory('ir_launch'),
        'launch',
        'assignment_1.launch.py'
    )

    return LaunchDescription([
        Node(
            package='group_32_assignment_1',
            namespace='init_pose_publisher',
            executable='init_pose_publisher',
            name='sim',
            arguments=['--ros-args', '--log-level', 'info']
        ),
        Node(
            package='group_32_assignment_1',
            namespace='group_32_assignment_1',
            executable='nav2pose',
            name='sim',
            ros_arguments=['--log-level', 'warn']
        ),
                Node(
            package='group_32_assignment_1',
            namespace='group_32_assignment_1',
            executable='manage_lifecycle_nodes',
            name='sim',
            ros_arguments=['--log-level', 'warn']
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(included_launch_file)
        )

    ])