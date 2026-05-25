cmake_minimum_required(VERSION 3.8)
project(ida_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

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

install(DIRECTORY launch config models worlds urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_to_thrust.py
  scripts/mission_eval_node.py
  scripts/motor_thrust_viewer.py
  scripts/sim_nav_converter.py
  scripts/sim_buoy_detector.py
  scripts/spawn_course_objects.py
  scripts/spawn_waypoint_markers.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
