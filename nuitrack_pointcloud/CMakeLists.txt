cmake_minimum_required(VERSION 3.8)
project(nuitrack_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(NUITRACK_SDK_INCLUDE_DIR /home/albmin2/Nuitrack/nuitrack-sdk-master/Nuitrack/include)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  ${NUITRACK_SDK_INCLUDE_DIR}
)
link_directories("/usr/local/lib/nuitrack")

add_executable(nuitrack_skeleton_pointcloud src/nuitrack_skeleton_pointcloud.cpp)
ament_target_dependencies(nuitrack_skeleton_pointcloud 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  PCL
  pcl_conversions
  angles
  tf2
  tf2_ros
  tf2_sensor_msgs)
target_link_libraries(nuitrack_skeleton_pointcloud nuitrack ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_SURFACE_LIBRARIES})

add_executable(nuitrack_skeleton_pointcloud_1 src/nuitrack_skeleton_pointcloud_1.cpp)
ament_target_dependencies(nuitrack_skeleton_pointcloud_1 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  PCL
  pcl_conversions
  angles
  tf2
  tf2_ros
  tf2_sensor_msgs)
target_link_libraries(nuitrack_skeleton_pointcloud_1 nuitrack ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_SURFACE_LIBRARIES})

add_executable(nuitrack_skeleton_pointcloud_2 src/nuitrack_skeleton_pointcloud_2.cpp)
ament_target_dependencies(nuitrack_skeleton_pointcloud_2 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  PCL
  pcl_conversions
  angles
  tf2
  tf2_ros
  tf2_sensor_msgs)
target_link_libraries(nuitrack_skeleton_pointcloud_2 nuitrack ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_SURFACE_LIBRARIES})


#Don't forget add executable name here
install(TARGETS
    nuitrack_skeleton_pointcloud
    nuitrack_skeleton_pointcloud_1
    nuitrack_skeleton_pointcloud_2
    DESTINATION lib/${PROJECT_NAME}) 


# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

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
