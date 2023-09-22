cmake_minimum_required(VERSION 3.8)
project(robot_control_part)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(angles REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(std_srvs REQUIRED)


# collision_objects.cpp part
add_executable(collision_objects src/collision_objects.cpp)
target_include_directories(collision_objects PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(collision_objects PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  collision_objects
  "moveit_ros_planning_interface"
  "rclcpp"
)


# handover_statemachine.cpp part
add_executable(handover_statemachine src/handover_statemachine.cpp)
target_include_directories(handover_statemachine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(handover_statemachine PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  handover_statemachine
  "moveit_ros_planning_interface"
  "rclcpp"
  "ur_msgs"
)


# pickplace_test.cpp part
add_executable(pickplace_test src/pickplace_test.cpp)
target_include_directories(pickplace_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pickplace_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  pickplace_test
  "moveit_ros_planning_interface"
  "rclcpp"
  "ur_msgs"
  "ur_client_library"
  "std_srvs"
)

# min_distance.cpp part
add_executable(min_distance src/min_distance.cpp)
target_include_directories(min_distance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(min_distance PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  min_distance
  "moveit_ros_planning_interface"
  "rclcpp"
  "ur_msgs"
  "sensor_msgs"
  "std_msgs"
  "PCL"
  "pcl_conversions"
)

# collision_avoidance_robot.cpp part
add_executable(collision_avoidance_robot src/collision_avoidance_robot.cpp)
target_include_directories(collision_avoidance_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(collision_avoidance_robot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  collision_avoidance_robot
  "moveit_ros_planning_interface"
  "rclcpp"
  "ur_msgs"
  "ur_client_library"
  "std_srvs"
)


# robot_jointsPosition.cpp part
add_executable(robot_jointsPosition src/robot_jointsPosition.cpp)
target_include_directories(robot_jointsPosition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_jointsPosition PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  robot_jointsPosition
  "moveit_ros_planning_interface"
  "rclcpp"
  "ur_msgs"
  "sensor_msgs"
  "std_msgs"
  "PCL"
  "pcl_conversions"
)


install(TARGETS 
  collision_objects
  handover_statemachine
  pickplace_test
  min_distance
  robot_jointsPosition
  collision_avoidance_robot
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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