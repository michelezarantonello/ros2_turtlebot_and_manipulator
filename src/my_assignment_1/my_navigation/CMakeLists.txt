cmake_minimum_required(VERSION 3.8)
project(my_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(init_pose_publisher src/init_pose_publisher.cpp)
ament_target_dependencies(init_pose_publisher rclcpp geometry_msgs nav2_msgs tf2 tf2_ros rclcpp_action std_msgs)  

add_executable(manage_localization_nodes src/manage_localization_nodes.cpp)
ament_target_dependencies(manage_localization_nodes rclcpp geometry_msgs nav2_msgs tf2 tf2_ros rclcpp_action std_msgs)  

add_executable(manage_navigation_nodes src/manage_navigation_nodes.cpp)
ament_target_dependencies(manage_navigation_nodes rclcpp geometry_msgs nav2_msgs tf2 tf2_ros rclcpp_action std_msgs) 

add_executable(nav2pose src/nav2pose.cpp)
ament_target_dependencies(nav2pose rclcpp geometry_msgs nav2_msgs tf2 tf2_ros rclcpp_action std_msgs) 

add_executable(table_detection src/table_detection.cpp)
ament_target_dependencies(table_detection rclcpp geometry_msgs nav2_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs sensor_msgs)  

install(TARGETS
  init_pose_publisher
  nav2pose
  manage_localization_nodes
  manage_navigation_nodes
  table_detection
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()