from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessIO
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import SetLaunchConfiguration

# def generate_launch_description():

#     ir_launch_file = os.path.join(
#         get_package_share_directory('ir_launch'),
#         'launch',
#         'assignment_2.launch.py'
#     )

#     apriltag_launch = os.path.join(
#         get_package_share_directory('group_32_apriltag_assignment_2'),
#         'launch',
#         'camera_36h11.launch.yml'
#     )

#     simulation = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(ir_launch_file)
#     )

#     camera_and_tags = IncludeLaunchDescription(
#         AnyLaunchDescriptionSource(apriltag_launch)
#     )

#     pre_grasp_pose_publisher = Node(
#        package='group_32_implementation_assignment_2',
#        executable='pre_grasp_pose_publisher',
#        name='pre_grasp_pose_publisher',
#        parameters=[{'use_sim_time': True}],
#        output='screen'
#     )
    
#     collision_objects = Node(
#         package='group_32_implementation_assignment_2',
#         executable='collision_objects',
#         name='collision_objects',
#         parameters=[{'use_sim_time': True}],
#         output='screen'
#     )



#     manipulation = Node(
#        package='group_32_implementation_assignment_2',
#        executable='manipulation',
#        name='manipulation',
#        parameters=[
#           moveit_config.to_dict(),
#           {'use_sim_time': True}],
#        output='screen'
#     )

 

#     return LaunchDescription([
#         SetLaunchConfiguration('use_sim_time', 'true'),
#         simulation,
#         TimerAction(period=13.0, actions=[ camera_and_tags ]),
#      #   TimerAction(period=22.0, actions=[ collision_objects ]),
#         TimerAction(period=35.0, actions=[ pre_grasp_pose_publisher ]),
#         TimerAction(period=55.0, actions=[ manipulation ])
#       ])  


#two ways for launching: the above commented one that do not work with cartesian path because that manipulation node do not have 
# the urdf/srdf available. The other way is the one below that requires or to create a urdf folder or to change ur_gripper.urdf.xacro
def generate_launch_description():
    
    ir_launch_file = os.path.join(get_package_share_directory('ir_launch'), 'launch', 'assignment_2.launch.py')
    apriltag_launch = os.path.join(get_package_share_directory('group_32_apriltag_assignment_2'), 'launch', 'camera_36h11.launch.yml')

    moveit_config_pkg = "ir_movit_config"
    moveit_config_dir = get_package_share_directory(moveit_config_pkg)
    xacro_path = os.path.join(moveit_config_dir, "config", "ur_gripper.urdf.xacro")
    srdf_path = os.path.join(moveit_config_dir, "config", "ur_gripper.srdf")
    moveit_config = (
        MoveItConfigsBuilder("ur_gripper", package_name=moveit_config_pkg)
        .robot_description(file_path=xacro_path)
        .robot_description_semantic(file_path=srdf_path)
        .to_moveit_configs()
    )

    simulation = IncludeLaunchDescription(PythonLaunchDescriptionSource(ir_launch_file))
    camera_and_tags = IncludeLaunchDescription(AnyLaunchDescriptionSource(apriltag_launch))

    pre_grasp_pose_publisher = Node(
        package='group_32_implementation_assignment_2',
        executable='pre_grasp_pose_publisher',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    manipulation = Node(
        package='group_32_implementation_assignment_2',
        executable='manipulation',
        name='manipulation',
        parameters=[
            moveit_config.to_dict(), 
            {'use_sim_time': True}
        ],
        output='screen'
    )
    collision_objects = Node(
        package='group_32_implementation_assignment_2',
        executable='collision_objects',
        name='collision_objects',
        parameters=[
            moveit_config.to_dict(), 
            {'use_sim_time': True}],
        output='screen'
    )

    color_detector = Node(
        package='color_detector',
        executable='color_detector',
        name='color_detector',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time':True}
        ],
        output='screen'
    )

    return LaunchDescription([
        SetLaunchConfiguration('use_sim_time', 'true'),
        simulation,
        TimerAction(period=13.0, actions=[camera_and_tags]),
        TimerAction(period=25.0, actions=[collision_objects]),
        TimerAction(period=35.0, actions=[pre_grasp_pose_publisher]),
        TimerAction(period=49.0, actions=[manipulation]),
        TimerAction(period=65.0, actions=[color_detector])
    ])