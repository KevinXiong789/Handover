
# Standart library:

# ament:
from ament_index_python import get_package_share_directory

# ROS-launch:
import launch
import launch_ros.actions
import launch.actions
import launch.launch_description_sources
import launch.substitutions



lop_parameters = [
    # Publish selection:
    {'name': 'publish_pose_images', 'default_value': 'True'},
    {'name': 'publish_2d_poses',    'default_value': 'False'},
    {'name': 'publish_3d_poses',    'default_value': 'False'},
    {'name': 'publish_3d_markers',  'default_value': 'True'},
    # Publisher topic name (/lop/marker_3d_marker):
    {'name': 'publisher_base_topic_name', 'default_value': 'lop'},
]

realsense_parameters = [
    {'name': 'realsense_camera_name',          'default_value': ''},
    {'name': 'realsense_camera_serial_number', 'default_value': ''},
]


def create_launch_parameters (parameters):
    '''Setup arguments of this launch file.'''

    return [launch.actions.DeclareLaunchArgument(parameter['name'], default_value=parameter['default_value'] )
            for parameter in parameters]


def create_launch_configurations (parameters):
    '''Read arguments of this launch file.'''

    return [ {parameter['name']: launch.substitutions.LaunchConfiguration(parameter['name'] ) }
            for parameter in parameters]



def generate_launch_description():

    # RealSense camera:
    realsense_camera_name =          launch.substitutions.LaunchConfiguration('realsense_camera_name')
    realsense_camera_serial_number = launch.substitutions.LaunchConfiguration('realsense_camera_serial_number')


    # Node instead of launch-file for easier parameter assignment:
    realsense_node = launch_ros.actions.Node(
        package='realsense2_camera',
        namespace=realsense_camera_name,
        name=realsense_camera_name,
        executable='realsense2_camera_node',
        parameters=[ {
            'camera_name':        realsense_camera_name,
            'serial_no':          ['"', realsense_camera_serial_number, '"'],  # Double quotes are required.
            'align_depth.enable': True,
            'pointcloud.enable':  True,
            'clip_distance':      5.0,  # Filter out depth-data past this distance to the camera, in meters.
            'depth_module.enable_auto_exposure': True,
            'enable_color': True,
            'enable_accel':  False,
            'enable_gyro':   False,
            'enable_infra1': False,
            'enable_infra2': False,
            'pointcloud.stream_filter': 2,
            #'rosbag_filename': '/ros2_ws/lop-rec-32',  # Does not work.
            #'base_frame_id': [realsense_camera_name, '_base_frame'],
            # 'spatial_filter.enable': True,
            # 'temporal_filter.enable': True,
            #'decimation_filter.enable': True,
            #'decimation_filter.magnitude':8
        } ],
        output='screen',
        emulate_tty=True,
    )


    # Replace camera node with rosbag recording:
    # rosbag_process = launch.actions.ExecuteProcess(
    #     cmd = 'ros2 bag play /ros2_ws/lop-rec-32 --loop --read-ahead-queue-size 100'.split(sep=' '),
    #     output='screen'
    # )


    # LOP input:
    lop_node = launch_ros.actions.Node(
        name=      'lop2',
        package=   'lop2',
        namespace= 'lop2',
        executable='lop_node',
        parameters=[
            *create_launch_configurations(parameters=lop_parameters),
            {'realsense_camera_name': realsense_camera_name},
        ]
    )

    
    

    # Transformation to the position of the RealSense camera:
    camera_base_tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['16.5', '2588', '0',
                   '0', '-0.236', '0',  # In radians, from -27°.
                   [realsense_camera_name, '_color_optical_frame'], [realsense_camera_name, '_base_frame'] ],
        output='screen'
    )
    
    tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0',
                   '0', '0', '0',
                   'world',  [realsense_camera_name, '_color_optical_frame'] ],
        output='screen'
    )

    '''
    # Transformation to the position of the RealSense camera:
    camera_base_tf_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.025', '1.1', '0.3',
                '2.0507621', '0.0174533', '-0.00872665', 
                'world', 'recCamera_depth_optical_frame'],
        output='screen'
    )
    '''
    

    return launch.LaunchDescription(initial_entities=[
        *create_launch_parameters(parameters=lop_parameters),
        *create_launch_parameters(parameters=realsense_parameters),
        
        lop_node,
        realsense_node,
        camera_base_tf_node,
        #tf_node,
        #realsense_process
        #rosbag_process
    ] )
