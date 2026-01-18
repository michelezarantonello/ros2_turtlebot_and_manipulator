To launch assignment_1 (tested on Ubuntu 24.04 and ROS2):
- correct branch to use: finally_exact_branch
- ir_2526 in ~ws_group_32_assignments/src is empty -> clone it from [https://github.com/PieroSimonet/ir_2526.git]
- open a new terminal 
- go to the workspace folder ~ws_group_32_assignments
- "colcon build"
- "source install/setup.bash"
- "ros2 launch group_32_launch my_launch.launch.py"
- when goal pose is reached, open a new terminal and source it (source install/setup.bash) 
- "ros2 run group_32_navigation table_detection" -> you will see position of the 3 tables printed in console

To launch assignment_2 (tested on Ubuntu 24.04 and ROS2):
- correct branch to use: main
- ir_2526 in ~ws_group_32_assignments/src is empty -> clone it from [https://github.com/PieroSimonet/ir_2526.git]
- open a new terminal 
- go to the workspace folder ~ws_group_32_assignments
- "colcon build"
- "source install/setup.bash"
- "ros2 launch group_32_launch_assignment_2 swap_cubes.launch.py"
