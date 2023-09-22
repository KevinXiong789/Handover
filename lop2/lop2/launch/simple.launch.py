
# Standart library:
import os

# ament:
from ament_index_python import get_package_share_directory

# ROS-launch:
import launch
import launch_ros.actions
import launch.actions
import launch.launch_description_sources

from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess



#realsense_d435_serial_number =   '937622073146'
realsense_d435_serial_number = '838212073332'
#realsense_d435i_serial_number =  '923322070787'
realsense_d435i_serial_number =  '117322070913'
realsense_d455_1_serial_number = '146222254681'
realsense_d455_2_serial_number = '215122251396'

realsense_camera_serial_number = realsense_d455_1_serial_number


publish_images =  True
publish_markers = True

'''
rviz_config_file_name = 'depth_only' if (not publish_images) and publish_markers \
						else 'image_only' if publish_images and (not publish_markers) \
						else 'marker'
'''

#rviz_config_file_name = 'with_ur5e'


def generate_launch_description():
	ld = launch.LaunchDescription()

	package_share_directory = get_package_share_directory(package_name='lop2')

	lop_node = launch_ros.actions.Node(
		name='lop',
		package='lop2',
		namespace='lop',
		executable='lop_node',
		parameters=[
			{'publish_images':  publish_images},
			{'publish_markers': publish_markers}
		]
	)
	ld.add_action(lop_node)

	'''
	rviz_node = launch_ros.actions.Node(
		name='rviz',
		package='rviz2',
		namespace='lop_rviz',
		executable='rviz2',
		arguments=['-d' + os.path.join(package_share_directory, 'config', f'{rviz_config_file_name}.rviz'), '--fixed-frame', 'world' ]
	)
	ld.add_action(rviz_node)
	'''
	
	realsense_launch = launch.actions.IncludeLaunchDescription(
		launch_description_source=launch.launch_description_sources.PythonLaunchDescriptionSource(
			launch_file_path=os.path.join(
				get_package_share_directory(package_name='realsense2_camera'), 'launch/rs_launch.py')
		),
		launch_arguments={
			'align_depth.enable': 'true',
			'pointcloud.enable': 'true',
			'serial_no': f"'{realsense_camera_serial_number}'",  # Double quotes are required.
			'clip_distance': '5.0',  # Filter out depth data past this distance to the camera, in meters.
			'config_file': os.path.join(package_share_directory, 'data', 'realsense-config.yaml'),
		}.items()
	)
	ld.add_action(realsense_launch)


	# Add static_transform_publisher, convert camera_link to world
	static_transform_publisher = launch_ros.actions.Node(
		package='tf2_ros',
		executable='static_transform_publisher',
		arguments=[ str(0.0), str(0.0), str(0.0), str(0.0), str(0.0), str(0.0), str(1.0), 'world', 'camera_link']
	)
	ld.add_action(static_transform_publisher)
	

	# Add execute process for launching ur_robot_driver
	ur_robot_driver_launch = launch.actions.ExecuteProcess(
		cmd=['ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py', 'ur_type:=ur10e', 'robot_ip:=192.168.1.102',
			'launch_rviz:=false'],
		output='screen'
	)
	ld.add_action(ur_robot_driver_launch)

	
	# Add execute process for launching ur_moveit_config
	ur_moveit_config_launch = launch.actions.ExecuteProcess(
		cmd=['ros2', 'launch', 'ur_moveit_config', 'ur_moveit.launch.py', 'ur_type:=ur10e', 'launch_rviz:=true'],
		output='screen'
	)
	ld.add_action(ur_moveit_config_launch)
	
	'''
	# Define a timer to trigger the execution of a C++ file after 10 seconds
	handover_after10 = TimerAction(period=10.0, actions=[
		ExecuteProcess(
			cmd=['ros2', 'run', 'pointcloud_processing', 'handover_T_inhand_detector'],
			output='screen'
		)
	])
	ld.add_action(handover_after10)
    '''
	
	return ld
	#return launch.LaunchDescription(initial_entities=[lop_node, realsense_launch, static_transform_publisher, ur_robot_driver_launch, ur_moveit_config_launch] )
	#return launch.LaunchDescription(initial_entities=[lop_node, rviz_node, realsense_launch, static_transform_publisher, ur_robot_driver_launch, ur_moveit_config_launch] )
