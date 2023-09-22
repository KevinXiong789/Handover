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

def generate_launch_description():

    
    rviz_node = launch_ros.actions.Node(
        name='rviz',
        package='rviz2',
        namespace='lop_rviz',
        executable='rviz2',
        #arguments=['-d' + os.path.join(package_share_directory, 'config', f'{rviz_config_file_name}.rviz') ]
    )
    
    nuitrack_node = launch_ros.actions.Node(
        package='nuitrack_pointcloud',
        executable='nuitrack_skeleton_pointcloud',
        output='screen'
    )

    nuitrack_1_node = launch_ros.actions.Node(
        package='nuitrack_pointcloud',
        executable='nuitrack_skeleton_pointcloud_1', # right camera, red, plug in the right usb
        output='screen'
    )

    nuitrack_2_node = launch_ros.actions.Node(
        package='nuitrack_pointcloud',
        executable='nuitrack_skeleton_pointcloud_2', # left camera, green, plug in the left usb
        output='screen'
    )


    static_transform_publisherT  = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.0041363', '0.2765040', '1.0475700',
                '1.53114265', '0.42689265', '0.03388265', # use recCamera_link Matrix in file "matrix3cameras.txt"
                'world','cameraT_link'],
        # x y z
        # yaw pitch roll
        output='screen'
    )
    
    static_transform_publisherR  = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.4614380', '0.2186506', '0.2227520',
                '0.75865274', '-0.00171671', '-0.01183805', # use recCameraR_link Matrix in file "matrix3cameras.txt"
                'world','cameraR_link'],
        # x y z
        # yaw pitch roll
        output='screen'
    )

    static_transform_publisherL  = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.4982635', '0.2166730', '0.2317470',
                '2.34878000', '0.00054563', '0.00011913', # use recCameraL_link Matrix in file "matrix3cameras.txt"
                'world','cameraL_link'],
        # x y z
        # yaw pitch roll
        output='screen'
    )
    
    # Add execute process for launching ur_robot_driver
    ur_robot_driver_launch = launch.actions.ExecuteProcess(
        cmd=['ros2', 'launch', 'ur_robot_driver', 'ur_control.launch.py', 'ur_type:=ur10e', 'robot_ip:=172.21.19.98',
             'launch_rviz:=false'],
        output='screen'
    )
    

    
    # Add execute process for launching ur_moveit_config
    ur_moveit_config_launch = TimerAction(period=5.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'launch', 'ur_moveit_config', 'ur_moveit.launch.py', 'ur_type:=ur10e', 'launch_rviz:=true'],
            output='screen'
        )
    ])
    


    # tranform the point cloud from camera frame to world
    pointcloud_transformer_launch = launch_ros.actions.Node(
        package='pointcloud_processing_nuitrack',
        executable='pointcloud_transformer',
        output='screen',
    )

    # average jointPosition from 3 cameras
    jointPosition_average_launch = launch_ros.actions.Node(
        package='pointcloud_processing_nuitrack',
        executable='jointPosition_transformer',
        output='screen',
    )

    collsion_objects_launch = TimerAction(period=6.0, actions=[
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
        nuitrack_node,
        nuitrack_1_node,
        nuitrack_2_node,
        static_transform_publisherT,
        static_transform_publisherR,
        static_transform_publisherL,
        ur_robot_driver_launch,
        ur_moveit_config_launch,
        pointcloud_transformer_launch,
        jointPosition_average_launch,
        collsion_objects_launch,
        robot_jointsPosition_launch,
        robot_self_filter_launch,
    ] )