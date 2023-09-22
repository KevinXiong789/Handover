
# Standart library:
import os

# ament:
from ament_index_python import get_package_share_directory

# ROS-launch:
import launch
import launch.actions
import launch.conditions
import launch.launch_description_sources



lop_parameters = [
    # Publish selection:
    {'name': 'publish_pose_images', 'default_value': 'True'},
    {'name': 'publish_2d_poses',    'default_value': 'False'},
    {'name': 'publish_3d_poses',    'default_value': 'False'},
    {'name': 'publish_3d_markers',  'default_value': 'True'},
]

realsense_parameters = [
    {'name': 'realsense_camera_serial_number_1', 'default_value': ''},
    {'name': 'realsense_camera_name_1', 'default_value': ''},

    {'name': 'realsense_camera_serial_number_2', 'default_value': ''},
    {'name': 'realsense_camera_name_2', 'default_value': ''},

    {'name': 'realsense_camera_serial_number_3', 'default_value': ''},
    {'name': 'realsense_camera_name_3', 'default_value': ''},

    {'name': 'realsense_camera_serial_number_4', 'default_value': ''},
    {'name': 'realsense_camera_name_4', 'default_value': ''},
]



def create_launch_parameters (parameters):
    return [launch.actions.DeclareLaunchArgument(parameter['name'], default_value=parameter['default_value'] )
            for parameter in parameters]


def create_launch_configurations (parameters):
    return {parameter['name']: launch.substitutions.LaunchConfiguration(parameter['name'] )
            for parameter in parameters}



def generate_launch_description():

    lop_launches = list()

    for index in range(1, 5):
        realsense_camera_name =          launch.substitutions.LaunchConfiguration(f'realsense_camera_name_{index}')
        realsense_camera_serial_number = launch.substitutions.LaunchConfiguration(f'realsense_camera_serial_number_{index}')

        lop_launch = launch.actions.IncludeLaunchDescription(
            launch_description_source=launch.launch_description_sources.PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(get_package_share_directory(package_name='lop2'), 'launch/lop.launch.py') ),
            condition=launch.conditions.LaunchConfigurationNotEquals(
                launch_configuration_name=f'realsense_camera_serial_number_{index}', expected_value=''),
            launch_arguments={
                **create_launch_configurations(parameters=lop_parameters),

                'publisher_base_topic_name': ['lop_', realsense_camera_name],  # Substitutions cannot be joined as strings.

                'realsense_camera_name': realsense_camera_name,
                'realsense_camera_serial_number': realsense_camera_serial_number,
            }.items()
        )

        lop_launches.append(lop_launch)


    return launch.LaunchDescription(initial_entities=[
        *create_launch_parameters(parameters=lop_parameters),
        *create_launch_parameters(parameters=realsense_parameters),
        *lop_launches,
    ] )
