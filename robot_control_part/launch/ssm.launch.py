
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

    # get min_distance between robot and human
    min_distance_launch = launch_ros.actions.Node(
        package='robot_control_part',
        executable='min_distance',
        output='screen',
    )

    # set the robot speed according to the distance
    jointPosition_transformer_launch = launch_ros.actions.Node(
        package='robot_control_part',
        executable='collision_avoidance_robot',
        output='screen',
    )
    
    # set grasp flag
    handover_grasping_detector_launch = launch_ros.actions.Node(
        package='pointcloud_processing',
        executable='handover_grasping_detector',
        output='screen',
    )


    return launch.LaunchDescription(initial_entities=[
        min_distance_launch,
        jointPosition_transformer_launch,
        handover_grasping_detector_launch,
    ] )
