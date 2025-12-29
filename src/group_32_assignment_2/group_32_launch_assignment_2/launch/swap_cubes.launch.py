from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ir_launch_file = os.path.join(
        get_package_share_directory('ir_launch'),
        'launch',
        'assignment_2.launch.py'
    )

    apriltag_launch = os.path.join(
        get_package_share_directory('apriltag_detector'),
        'launch',
        'camera_36h11.launch.yml'
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ir_launch_file)
    )

    camera_and_tags = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(apriltag_launch)
    )

   #  init_pose_publisher = Node(
   #     package='group_32_navigation',
   #     executable='init_pose_publisher',
   #     name='init_pose_publisher'
   # )

 

    return LaunchDescription([
        simulation,
        TimerAction(period=13.0, actions=[ camera_and_tags ]),
   #     TimerAction(period=44.0, actions=[ init_pose_publisher ])
      ])  