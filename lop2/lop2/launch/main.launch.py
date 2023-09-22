
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

realsense_d435_1_serial_number = '937622073146' # right
realsense_d435_2_serial_number = '838212073567' # top
realsense_d435_3_serial_number = '838212073332' # left

realsense_d435i_1_serial_number = '117322070913'
realsense_d435i_2_serial_number = '117322070475'

realsense_d455_1_serial_number = '213522254445'
realsense_d455_2_serial_number = '215122251396'
realsense_d455_3_serial_number = '146222253769'
realsense_d455_4_serial_number = '146222254681'


lop_parameters = {
    # Publish selection:
    'publish_pose_images': 'True',
    'publish_2d_poses'   : 'False',
    'publish_3d_poses'   : 'False',
    'publish_3d_markers' : 'True',
}

# D455 can run, but the point cloud is too slow, only 17 fps
# left D435i run alone, but the point cloud is slow, only 20 fps or less
# top D435 run alone, 27.5 fps, because the point cloud in this direction are more
# right D435 run alone, 29.5 fps, because the point cloud in this direction are less
## all three cameras run openpose tracker
## three cameras run together, top: 17 fps, left: 9 fps, right: 19 fps (no pointcloud_transformer)
## three cameras run together, top: 11 fps, left: 6 fps, right: 13 fps (with pointcloud_transformer)

# now left D435i is replaced by a D435, better
# left D435 run alone, 29.5 fps
# top D435 run alone, 27.5 fps, because the point cloud in this direction are more
# right D435 run alone, 29.5 fps, because the point cloud in this direction are less
## all three cameras run openpose tracker
## three cameras run together, top: 19 fps, left: 24 fps, right: 20 fps (no pointcloud_transformer)
## three cameras run together, top: 13 fps, left: 21 fps, right: 18 fps (with pointcloud_transformer)

# If the serial number is an empty string, then no node is launched:
realsense_parameters = {
    'realsense_camera_serial_number_1': realsense_d435_2_serial_number,
    'realsense_camera_name_1':          'recCamera',
    #'realsense_camera_serial_number_1': '',
    #'realsense_camera_name_1':          '',

    'realsense_camera_serial_number_2': realsense_d435_1_serial_number,
    'realsense_camera_name_2':          'recCameraR',
    #'realsense_camera_serial_number_2': '',
    #'realsense_camera_name_2':          '',

    'realsense_camera_serial_number_3': realsense_d435_3_serial_number,
    'realsense_camera_name_3':          'recCameraL',
    #'realsense_camera_serial_number_3': '',
    #'realsense_camera_name_3':          '',

    #'realsense_camera_serial_number_4': realsense_d435i_1_serial_number,
    #'realsense_camera_name_4':          'recCameraT',
    'realsense_camera_serial_number_4': '',
    'realsense_camera_name_4':          '',
}


rviz_config_file_name = 'comparison'



def generate_launch_description():

    package_share_directory = get_package_share_directory(package_name='lop2')

    
    rviz_node = launch_ros.actions.Node(
        name='rviz',
        package='rviz2',
        namespace='lop_rviz',
        executable='rviz2',
        #arguments=['-d' + os.path.join(package_share_directory, 'config', f'{rviz_config_file_name}.rviz') ]
    )
    


    static_transform_publisher  = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[#'-0.03125', '0.2453125', '1.12',
                #'1.5831853', '0.4799655', '0.00872664',
                #'-0.0375', '0.3', '1.1125',           ## original manual parameter
                #'1.579523', '0.4799655', '0.0087266',
                #'0.0124431', '0.314105', '1.08615', ## automatic parameter
                #'-1.57218', '2.68524', '3.11531',
                '-0.0041363', '0.276504', '1.04757',           ## d435 automatic parameter
                '-1.61045', '2.7147', '-3.10771',
                'world','recCamera_link'],
        output='screen'
    )
 # new
 #'0.0203125', '0.29375', '1.1171875',
 #'1.579523', '0.4799655', '0.0'
 # new2
 #'-0.0375', '0.3', '1.1125',
 #'1.579523', '0.4799655', '0.0087266',

    static_transform_publisherL  = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[#'0.484201', '0.207298', '0.231747',
                #'2.34878', '0.000545628', '0.000119133',  ## automatic parameter
                '0.4982635', '0.216673', '0.231747',
                '2.34878', '0.000545628', '0.000119133', ## manual adjusted parameter
                'world','recCameraL_link'],
        output='screen'
    )

    static_transform_publisherR  = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[#'-0.464563', '0.227635', '0.238377',
                #'-2.38294', '3.12197', '-3.12466', ## automatic parameter
                '-0.461438', '0.218650625', '0.222752',
                '-2.382939916', '3.14330936', '-3.1534307', ## manual adjusted parameter
                'world','recCameraR_link'],
        output='screen'
    )

    static_transform_publisherT  = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0826202', '0.868798', '1.08253',
                '-1.56002', '1.5664', '0.016588', ## automatic parameter top_2, not very good, still can be used in specific area 
                #'0.0582845', '0.901819', '1.07555',
                #'-1.54139', '1.58798', '0.0433393', ## automatic parameter top, bad
                'world','recCameraT_link'],
        output='screen'
    )

    
    # Add execute process for launching ur_robot_driver
    ur_robot_driver_launch = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py', 'ur_type:=ur10e', 
             'robot_ip:=172.21.19.98',
             #'robot_ip:=192.168.1.102',
             'launch_rviz:=false'],
        output='screen'
    )
    

    
    # Add execute process for launching ur_moveit_config
    ur_moveit_config_launch = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ur_moveit_config', 'ur_moveit.launch.py', 'ur_type:=ur10e', 'launch_rviz:=True'],
            output='screen'
        )
    ])
    

    lop_launch = launch.actions.IncludeLaunchDescription(
        launch_description_source=launch.launch_description_sources.PythonLaunchDescriptionSource(
            launch_file_path=os.path.join(get_package_share_directory(package_name='lop2'), 'launch/run.launch.py') ),
        launch_arguments={
            **lop_parameters,
            **realsense_parameters,
        }.items()
    )
    '''
    # tranform the point cloud from camera frame to world
    pointcloud_transformer_launch = TimerAction(period=10.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'pointcloud_processing', 'pointcloud_transformer'],
            output='screen',
        )
    ])

    # tranform the human position from camera frame to world
    jointPosition_transformer_launch = TimerAction(period=10.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'pointcloud_processing', 'jointPosition_transformer'],
            output='screen',
        )
    ])
    '''

    # tranform the point cloud from camera frame to world
    pointcloud_transformer_launch = launch_ros.actions.Node(
        package='pointcloud_processing',
        executable='pointcloud_transformer',
        output='screen',
    )

    # average jointPosition from 3 cameras
    jointPosition_transformer_launch = launch_ros.actions.Node(
        package='pointcloud_processing',
        executable='jointPosition_transformer',
        output='screen',
    )
    

    collsion_objects_launch = TimerAction(period=8.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robot_control_part', 'collsion_objects.launch.py'],
            output='screen'
        )
    ])

    # publish the robot joints position
    robot_jointsPosition_launch = TimerAction(period=8.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'launch', 'robot_control_part', 'robot_jointsPosition.launch.py'],
            output='screen'
        )
    ])

    robot_self_filter_launch = TimerAction(period=12.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'run', 'pointcloud_processing', 'robot_self_filter'],
            output='screen',
        )
    ])



    return launch.LaunchDescription(initial_entities=[
        #rviz_node,
        lop_launch,
        static_transform_publisher, 
        static_transform_publisherL,
        static_transform_publisherR,
        #static_transform_publisherT,
        ur_robot_driver_launch,
        ur_moveit_config_launch,
        pointcloud_transformer_launch,
        jointPosition_transformer_launch,
        collsion_objects_launch,
        robot_jointsPosition_launch,
        robot_self_filter_launch,
    ] )
