from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import AnyLaunchDescriptionSource
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
        get_package_share_directory('my_apriltag'),
        'launch',
        'camera_36h11.launch.yml'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ir_launch_file)
    )

    camera_and_tags = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(apriltag_launch)
    )

    init_pose_publisher = Node(
        package='my_navigation',
        executable='init_pose_publisher',
        name='init_pose_publisher'
    )

    localization_manager = Node(
        package='my_navigation',
        executable='manage_localization_nodes',
        name='manage_localization_nodes'
    )
    navigation_manager = Node(
        package='my_navigation',
        executable='manage_navigation_nodes',
        name='manage_navigation_nodes'
    )

    nav2_goal = Node(
        package='my_navigation',
        executable='nav2pose',
        name='nav2pose'
    )

    return LaunchDescription([
        simulation,
        TimerAction(period=10.0, actions=[ camera_and_tags ]),
        TimerAction(period=30.0, actions=[ localization_manager ]),
        TimerAction(period=44.0, actions=[ init_pose_publisher ]),
        TimerAction(period=66.0, actions=[ navigation_manager ]),
        TimerAction(period=90.0, actions=[ nav2_goal ]),
    ])