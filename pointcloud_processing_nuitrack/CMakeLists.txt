cmake_minimum_required(VERSION 3.8)
project(pointcloud_processing_nuitrack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#handover_T_inhand_detector.cpp part
add_executable(handover_T_inhand_detector src/handover_T_inhand_detector.cpp)
ament_target_dependencies(handover_T_inhand_detector
  rclcpp 
  sensor_msgs
  std_msgs
  PCL
  pcl_conversions
  angles

)
target_link_libraries(handover_T_inhand_detector ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_SURFACE_LIBRARIES})


#pointcloud_transformer.cpp part
add_executable(pointcloud_transformer src/pointcloud_transformer.cpp)
ament_target_dependencies(pointcloud_transformer
  rclcpp 
  sensor_msgs
  std_msgs
  PCL
  pcl_conversions
  angles
  tf2
  tf2_ros
  tf2_sensor_msgs

)
target_link_libraries(pointcloud_transformer ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_SAMPLE_CONSENSUS_LIBRARIES} ${PCL_SURFACE_LIBRARIES})

#jointPosition_transformer.cpp part
add_executable(jointPosition_transformer src/jointPosition_transformer.cpp)
ament_target_dependencies(jointPosition_transformer
  rclcpp 
  sensor_msgs
  std_msgs
  angles
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
)

#Don't forget add executable name here
install(TARGETS
    handover_T_inhand_detector
    pointcloud_transformer
    jointPosition_transformer
    DESTINATION lib/${PROJECT_NAME}) 


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
