from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ir_launch_file = os.path.join(
        get_package_share_directory('ir_launch'),
        'launch',
        'assignment_1.launch.py'
    )

    apriltag_launch = os.path.join(
        get_package_share_directory('group_32_apriltag'),
        'launch',
        'apriltag.launch.yml'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ir_launch_file)
    )

    camera_and_tags = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(apriltag_launch)
    )

    init_pose_publisher = Node(
        package='group_32_assignment_1',
        executable='init_pose_publisher',
        name='init_pose_publisher'
    )

    lifecycle_manager = Node(
        package='group_32_assignment_1',
        executable='manage_lifecycle_nodes',
        name='lifecycle_manager'
    )

    nav2_goal = Node(
        package='group_32_assignment_1',
        executable='nav2pose',
        name='nav2pose'
    )

    return LaunchDescription([
        simulation,
        TimerAction(period=5.0, actions=[ camera_and_tags ]),
        TimerAction(period=12.0, actions=[ lifecycle_manager ]),
        TimerAction(period=18.0, actions=[ init_pose_publisher ]),
        TimerAction(period=22.0, actions=[ nav2_goal ]),
    ])
