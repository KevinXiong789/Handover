
# Standart library:
from typing import Union
import pathlib
import math
from typing import Iterable

# Installed library:
import cv2
import torch
import numpy as np
import numpy.linalg as np_linalg

# ROS libraries:
import rclpy
import rclpy.node

from influxdb_client import InfluxDBClient, Point
import time
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PointStamped



import std_msgs.msg
import sensor_msgs.msg
import visualization_msgs.msg
import geometry_msgs.msg
import cv_bridge
import image_geometry
import message_filters

import ament_index_python.packages

# Custom ROS Messages:
import lop2_interfaces.msg

# Local libraries:
from .with_mobilenet import PoseEstimationWithMobileNet
from .keypoints import extract_keypoints, group_keypoints, BODY_PARTS_KPT_IDS
from .load_state import load_state
from .pose import Pose, track_poses
from .val import normalize, pad_width
from .one_euro_filter import OneEuroFilter



# OpenCV uses BGR-order:
OPENCV_COLORS = {
    'red':     (  0,   0, 255),
    'green':   (  0, 255,   0),
    'blue':    (255,   0,   0),
    'cyan':    (255, 255,   0),
    'magenta': (255,   0, 255),
    'yellow':  (  0, 255, 255)
}

ROS_COLORS = {
    'blue':    std_msgs.msg.ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
    'green':   std_msgs.msg.ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
    'cyan':    std_msgs.msg.ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
    'red':     std_msgs.msg.ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
    'magenta': std_msgs.msg.ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),
    'yellow':  std_msgs.msg.ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
}

MARKER_SIZE = geometry_msgs.msg.Vector3(x=0.1, y=0.1, z=0.1)
#MIN_KEYPOINT_DISTANCE_TO_CAMERA = 0.05
USE_CPU = False

USE_FILTERS = False
CONFIDENCE_THRESHOLD = 10

# ******************************************************************************
#set necessary info to connect InfluxDBClient
influxdb_client = InfluxDBClient(url="http://localhost:8086", \
                                token="mxxi_ghOGWNX4h5ZTPuUTox1rHP_n3VzyevupqvIKzfBdqIr9AyjJAb106xiEAG80yju0MGG7QI661b-lxE7XQ==", \
                                org="Pose_Postion")
influxdb_write_api = influxdb_client.write_api()
# ******************************************************************************


class Pose3D ():

    def __init__ (self, confidence=None, id=None, previous_pose=None) -> None:
        self.points = np.empty(shape=(Pose.number_of_keypoints, 3), dtype=np.float64)

        self.confidence = -1 if (confidence is None) else confidence
        self.id = 0 if (id is None) else id

        beta = 0.00      # Default: 0.05
        min_cutoff = 10  # Default: 1

        # Reuse filter if previous pose has valid filter:
        if (not USE_FILTERS) or (previous_pose is None):
            self.filters = [OneEuroFilter(beta=beta, min_cutoff=min_cutoff) ] * Pose.number_of_keypoints
        else:
            self.filters = [filter if (not any(np.isnan(point) ) ) else OneEuroFilter(beta=beta, min_cutoff=min_cutoff)
                            for point, filter in zip(previous_pose.points, previous_pose.filters)  ]


    def to_message (self) -> lop2_interfaces.msg.Pose3D:

        message = lop2_interfaces.msg.Pose3D()
        message.id = int(self.id)
        message.confidence = float(self.confidence)

        # Pose3D has a fixed size array of keypoints, appending does not work:
        for index, point in enumerate(self.points):
            message_point = message.keypoints[index]
            (message_point.x, message_point.y, message_point.z) = (float(dimension) for dimension in point)  # Casting is required for Numpy array.

        return message



class Lop (rclpy.node.Node):

    def __init__ (self) -> None:
        super().__init__(node_name='lop_node')

        # Parameters:
        self.publish_pose_images = self.get_ros_parameter(parameter_name='publish_pose_images', type=bool)
        self.publish_2d_poses =    self.get_ros_parameter(parameter_name='publish_2d_poses',    type=bool)
        self.publish_3d_poses =    self.get_ros_parameter(parameter_name='publish_3d_poses',    type=bool)
        self.publish_3d_markers =  self.get_ros_parameter(parameter_name='publish_3d_markers',  type=bool)

        self.publisher_base_topic_name = self.get_ros_parameter(parameter_name='publisher_base_topic_name', type=str)

        self.realsense_camera_name = self.get_ros_parameter(parameter_name='realsense_camera_name')
        self.frame_id = f'{self.realsense_camera_name}_color_optical_frame'


        # CV bridge:
        self.bridge = cv_bridge.CvBridge()


        # Message synchronisation:
        subscribers = list()

        subscribers.append(message_filters.Subscriber(self, sensor_msgs.msg.Image, f'/{self.realsense_camera_name}/color/image_raw') )
        subscribers.append(message_filters.Subscriber(self, sensor_msgs.msg.CameraInfo, f'/{self.realsense_camera_name}/color/camera_info') )

        if self.publish_3d_poses or self.publish_3d_markers:
            subscribers.append(message_filters.Subscriber(self, sensor_msgs.msg.Image, f'/{self.realsense_camera_name}/aligned_depth_to_color/image_raw') )

        time_synchronizer = message_filters.TimeSynchronizer(fs=subscribers, queue_size=10)
        time_synchronizer.registerCallback(self.callback)

        # ************************************************************************************************
        # Publisher for handposition
        #self.handposition_publisher = self.create_publisher(Float32MultiArray, f'/{self.publisher_base_topic_name}/hand_position', 10)
        # Publisher upper limb joint positions
        self.jointsposition_publisher = self.create_publisher(Float32MultiArray, f'/{self.publisher_base_topic_name}/limb_joints_position', 10)
        # ************************************************************************************************

        # Publishers:
        self.create_publishers()

        # Pose tracking:
        self.previous_3d_poses = list()

        # Visualisation:
        self.previous_marker_colors = dict()


        # Neural network setup:
        self.setup_neural_network()


    def get_ros_parameter (self, parameter_name: str, type=str) -> Union[str, bool]:

        if type not in (bool, str):
            raise ValueError(f'get_ros_parameter() only accepts: str or bool, given: {type}')

        self.declare_parameter(name=parameter_name, value=('' if type is str else False) )

        parameter_value = self.get_parameter(name=parameter_name).value


        self.get_logger().info(f"Loading parameter: '{parameter_name}': '{parameter_value}'.")

        return parameter_value


    def create_publishers (self) -> None:

        if self.publish_pose_images:
            self.image_info_publisher = self.create_publisher(topic=f'/{self.publisher_base_topic_name}/camera_info',
                                                              msg_type=sensor_msgs.msg.CameraInfo, qos_profile=10)

            self.pose_image_publisher = self.create_publisher(topic=f'/{self.publisher_base_topic_name}/pose_image',
                                                              msg_type=sensor_msgs.msg.Image, qos_profile=10)

        if self.publish_2d_poses:
            self.pose_2d_publisher = self.create_publisher(topic=f'/{self.publisher_base_topic_name}/pose_2d',
                                                           msg_type=lop2_interfaces.msg.Pose2DArray, qos_profile=10)

        if self.publish_3d_poses:
            self.pose_3d_publisher = self.create_publisher(topic=f'/{self.publisher_base_topic_name}/pose_3d',
                                                           msg_type=lop2_interfaces.msg.Pose3DArray, qos_profile=10)

        if self.publish_3d_markers:
            self.marker_3d_publisher = self.create_publisher(topic=f'/{self.publisher_base_topic_name}/marker_3d',
                                                             msg_type=visualization_msgs.msg.MarkerArray, qos_profile=10)


    def setup_neural_network (self) -> None:
        self.net = PoseEstimationWithMobileNet()

        share_directory_path = pathlib.Path(ament_index_python.packages.get_package_share_directory(package_name='lop2') )
        checkpoint_file_path = share_directory_path / 'data' / 'checkpoint_iter_370000.pth'
        checkpoint = torch.load(f=checkpoint_file_path, map_location='cpu')
        load_state(self.net, checkpoint)

        self.net = self.net.eval()

        if not USE_CPU:
            self.net = self.net.cuda()

        self.stride = 8
        self.upsample_ratio = 4
        self.previous_poses = []
        self.height_size = 256
        self.track = True
        self.smooth = True


    def callback (self, camera_message: sensor_msgs.msg.Image, camera_info_message: sensor_msgs.msg.CameraInfo,
                  depth_message: sensor_msgs.msg.Image = None) -> None:

        cv_image = self.bridge.imgmsg_to_cv2(img_msg=camera_message, desired_encoding='bgr8')

        poses = self.poses_from_lop(image=cv_image)

        confidence_string = ' | '.join(f'{pose.id}: {pose.confidence}' for pose in poses)
        #self.get_logger().info(f'{self.realsense_camera_name} > Confidences >> {confidence_string}')


        poses_2d = tuple(pose for pose in poses if (pose.confidence > CONFIDENCE_THRESHOLD) )


        if self.publish_pose_images:
            self.draw_keypoints(image=cv_image, poses=poses_2d)
            image_message = self.bridge.cv2_to_imgmsg(cvim=cv_image, encoding='bgr8',
                                                      header=camera_info_message.header)
            self.pose_image_publisher.publish(image_message)

            self.image_info_publisher.publish(camera_info_message)


        if self.publish_2d_poses:
            self.pose_2d_publisher.publish(msg=self.create_pose_array(poses=poses_2d, array_type=lop2_interfaces.msg.Pose2DArray) )


        if self.publish_3d_poses or self.publish_3d_markers:
            poses_3d = self.calculate_3d_poses(depth_message, camera_info_message, poses_2d)

            if self.publish_3d_poses:
                self.pose_3d_publisher.publish(msg=self.create_pose_array(poses=poses_3d, array_type=lop2_interfaces.msg.Pose3DArray) )

            if self.publish_3d_markers:
                self.marker_3d_publisher.publish(msg=self.create_pose_marker_array(poses=poses_3d) )


    def create_pose_array (self, poses, array_type):
        poses_message = array_type()

        for pose in poses:
            poses_message.poses.append(pose.to_message() )

        return poses_message


    def draw_keypoints (self, image: np.ndarray, poses: Iterable[Pose] ):
        for pose in poses:
            #self.get_logger().info(f'Pose [Confidence: {pose.confidence}]\n: {pose.keypoints}')

            for point in (point for point in pose.keypoints if not any( (coordinate < 0) for coordinate in point) ):
                cv2.circle(img=image, center=tuple(point), radius=15, color=OPENCV_COLORS['magenta'], thickness=-1)


            for keypoint_a_index, keypoint_b_index in BODY_PARTS_KPT_IDS[:-2]:

                if any( any( (coordinate < 0) for coordinate in pose.keypoints[index] ) for index in (keypoint_a_index, keypoint_b_index) ):
                    continue

                cv2.line(img=image, thickness=10, pt1=tuple(pose.keypoints[keypoint_a_index] ),
                         pt2=tuple(pose.keypoints[keypoint_b_index] ), color=OPENCV_COLORS['yellow'] )


    def calculate_3d_poses (self, depth_message, camera_info_message, poses) -> None:

        depth_image = self.bridge.imgmsg_to_cv2(img_msg=depth_message, desired_encoding='16UC1')

        pinhole_camera_model = image_geometry.PinholeCameraModel()
        pinhole_camera_model.fromCameraInfo(msg=camera_info_message)

        poses_3d = list()

        for pose_idx, pose_2d in enumerate(poses):
        #for pose_2d in poses:
            previous_3d_pose = next( (pose for pose in self.previous_3d_poses if (pose.id == pose_2d.id) ), None)
            pose_3d = Pose3D(confidence=pose_2d.confidence, id=pose_2d.id, previous_pose=previous_3d_pose)

            for pixel_coordinates, point_3d, point_3d_filter in zip(pose_2d.keypoints, pose_3d.points, pose_3d.filters):

                if any( (coordinate < 0) for coordinate in pixel_coordinates):  # No keypoint detected:
                    point_3d[:] = np.full(shape=3, fill_value=np.nan, dtype=np.float64)
                    continue

                unit_vector_3d = pinhole_camera_model.projectPixelTo3dRay(uv=pixel_coordinates)
                depth = (depth_image[tuple(pixel_coordinates[::-1] ) ] / 1000)  # Millimeter to meter.

                point_3d[:] = np.array(object=tuple( (element * depth) for element in unit_vector_3d), dtype=np.float64)

                if USE_FILTERS:
                    point_3d[:] = point_3d_filter(point_3d)

            poses_3d.append(pose_3d)
        
            '''
            keypoint_ID = 1
            point_name = f"Pose{pose_idx}_Neck_Openpose"
            people_ID = pose_idx + 1
            #keypoint_ID = 4
            #point_name = "right_hand_Openpose"

            if (pose_3d.points[keypoint_ID] is None) or (pose_3d.points[keypoint_ID][0]==0 and pose_3d.points[keypoint_ID][1]==0 and pose_3d.points[keypoint_ID][2]==0):
                pass
                
            else:
                distance_to_zero = math.sqrt((pose_3d.points[keypoint_ID][0]**2)+(pose_3d.points[keypoint_ID][1]**2)+(pose_3d.points[keypoint_ID][2]**2))
                timestamp = int(time.time()*1000)
                data_point = Point(point_name) \
                            .field("x", pose_3d.points[keypoint_ID][0]) \
                            .field("y", pose_3d.points[keypoint_ID][1]) \
                            .field("d", pose_3d.points[keypoint_ID][2]) \
                            .field("distance", distance_to_zero) \
                            .field("people_ID", people_ID) \
                            .time(timestamp,"ms")
                influxdb_write_api.write(bucket="OpenPose_automatica", record=data_point)
            '''
            # **********get right wrist and elbow position and publish***********************************
            '''
            if (pose_3d.points[4] is not None) and (pose_3d.points[3] is not None) and (people_ID == 1):
                msg = Float32MultiArray()
                msg.data = [pose_3d.points[4][0], pose_3d.points[4][1], pose_3d.points[4][2], pose_3d.points[3][0], pose_3d.points[3][1], pose_3d.points[3][2]]
                self.handposition_publisher.publish(msg)
            '''

            '''
            # Define the transformation matrix using the provided values
                
            #tx, ty, tz = -0.031215, 0.25, 1.12
            #tx, ty, tz = -0.011215, 0.25, 1.1
            tx, ty, tz = 0.0215917, 0.3001180, 1.1122583

            #yaw, pitch, roll = np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(-117.5)
            yaw = 0.00460947
            pitch = 0.00821774
            roll = -2.04713092

            # create rotation matrix
            R_yaw = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
            R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
            R_roll = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])

            R = R_yaw @ R_pitch @ R_roll

            # create transformation matrix
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = [tx, ty, tz]
            
            if (pose_3d.points[4] is not None) and (pose_3d.points[3] is not None) and (people_ID == 1):
                
                
                # apply transformation
                point4 = np.array([pose_3d.points[4][0], pose_3d.points[4][1], pose_3d.points[4][2], 1])
                point3 = np.array([pose_3d.points[3][0], pose_3d.points[3][1], pose_3d.points[3][2], 1])
                
        
                transformed_point4 = np.matmul(T, point4)
                transformed_point3 = np.matmul(T, point3)

                msg = Float32MultiArray()
                msg.data = [transformed_point4[0], transformed_point4[1], transformed_point4[2], transformed_point3[0], transformed_point3[1], transformed_point3[2]]
                self.handposition_publisher.publish(msg)
            '''
            # *******************************************************************************************

            # *********get upper limb Joint point Position and publish***********************************
            
            data_to_publish = []
            for i in range(8): # upper limb Joint Point
                if pose_3d.points[i] is not None:
                    point = np.array([pose_3d.points[i][0], pose_3d.points[i][1], pose_3d.points[i][2], 1])
                    #transformed_point = np.matmul(T, point)
                    #data_to_publish.extend(transformed_point[:3])
                    data_to_publish.extend(point[:3])

                else:
                    data_to_publish.extend([float('nan'), float('nan'), float('nan')])

            msg = Float32MultiArray()
            msg.data = data_to_publish
            self.jointsposition_publisher.publish(msg)
            #self.get_logger().info('Joint positions published.')  
            # ********************************************************************************************

        if not poses_3d:
            msg = Float32MultiArray()
            msg.data = [100.0, 100.0, 100.0]
            self.jointsposition_publisher.publish(msg)
            #self.get_logger().info('No valid 3D poses detected.')

        

        self.previous_3d_poses = poses_3d


        return poses_3d


    @staticmethod
    def validate_point (point: np.ndarray) -> float:

        if any(np.isnan(point) ):
            return False


        distance = np_linalg.norm(x=point, ord=2)

        #return (distance > MIN_KEYPOINT_DISTANCE_TO_CAMERA)
        return (distance > 0)


    def create_pose_marker_array (self, poses) -> None:
        # visualization_msgs.msg.Marker.LINE_LIST only uses the x-component, for line width:
        marker_scale = geometry_msgs.msg.Vector3(x=0.01)
        time_stamp = self.get_clock().now().to_msg()

        last_marker_colors = self.previous_marker_colors  # ID: color name.
        self.previous_marker_colors = dict()  # Reset.

        marker_array = visualization_msgs.msg.MarkerArray()

        for pose in poses:
            marker_color_name = last_marker_colors[pose.id] if pose.id in last_marker_colors.keys() \
                                else next( (color_name for color_name in tuple(ROS_COLORS.keys() )
                                            if color_name not in self.previous_marker_colors.values() ), None)

            if marker_color_name is None:  # Use the least common color if all colors are in use:
                marker_color_name = min(self.previous_marker_colors.values(), key=tuple(self.previous_marker_colors.values() ).count)

            self.previous_marker_colors[pose.id] = marker_color_name

            # Exclude bodyparts ear-to-shoulder, the last two:
            # Double list-comprehension to flatten the result (output two results):
            marker_points = tuple(points for keypoint_a_index, keypoint_b_index in BODY_PARTS_KPT_IDS[:-2]
                                  for points in (pose.points[keypoint_a_index], pose.points[keypoint_b_index] )
                                  if all(self.validate_point(pose.points[index] )
                                         for index in (keypoint_a_index, keypoint_b_index) ) )

            marker_array.markers.append(self.create_marker(index=pose.id, points=marker_points, time_stamp=time_stamp,
                                                           color=ROS_COLORS[marker_color_name], frame_id=self.frame_id, scale=marker_scale,
                                                           type=visualization_msgs.msg.Marker.LINE_LIST) )

        return marker_array


    def create_marker (self, index, points, frame_id, time_stamp, type=visualization_msgs.msg.Marker.POINTS,
                       color=ROS_COLORS['red'], scale=MARKER_SIZE) -> visualization_msgs.msg.Marker:

        marker = visualization_msgs.msg.Marker()

        marker.id = index
        marker.ns = 'lop'
        marker.color = color
        marker.action = visualization_msgs.msg.Marker.ADD
        marker.type = type
        marker.scale = scale
        marker.pose.orientation.w = 1.0
        marker.header.stamp = time_stamp
        marker.header.frame_id = frame_id
        marker.lifetime.sec = 1  # Seconds.
        marker.points = tuple(geometry_msgs.msg.Point(x=float(point[0] ), y=float(point[1] ), z=float(point[2] ) )
                              for point in points)

        return marker


    def infer_fast (self, net, img, net_input_height_size, stride, upsample_ratio, cpu,
                    pad_value=(0, 0, 0), img_mean=np.array( [128, 128, 128], np.float32),
                    img_scale=np.float32(1/256) ):

        scale = net_input_height_size / img.shape[0]

        scaled_img = cv2.resize(src=img, dsize=(0, 0), fx=scale, fy=scale,
                                interpolation=cv2.INTER_LINEAR)
        scaled_img = normalize(scaled_img, img_mean, img_scale)
        min_dims = [net_input_height_size, max(scaled_img.shape[1], net_input_height_size) ]
        padded_img, pad = pad_width(scaled_img, stride, pad_value, min_dims)

        tensor_img = torch.from_numpy(padded_img).permute(2, 0, 1).unsqueeze(0).float()
        if not cpu:
            tensor_img = tensor_img.cuda()

        stages_output = net(tensor_img)

        stage2_heatmaps = stages_output[-2]
        heatmaps = np.transpose(a=stage2_heatmaps.squeeze().cpu().data.numpy(), axes=(1, 2, 0) )
        heatmaps = cv2.resize(src=heatmaps, dsize=(0, 0), fx=upsample_ratio, fy=upsample_ratio,
                              interpolation=cv2.INTER_CUBIC)

        stage2_pafs = stages_output[-1]
        pafs = np.transpose(a=stage2_pafs.squeeze().cpu().data.numpy(), axes=(1, 2, 0) )
        pafs = cv2.resize(src=pafs, dsize=(0, 0), fx=upsample_ratio, fy=upsample_ratio,
                          interpolation=cv2.INTER_CUBIC)

        return heatmaps, pafs, scale, pad


    def poses_from_lop (self, image):
        # PAF: Part affinity field.

        heatmaps, pafs, scale, pad = self.infer_fast(self.net, image, self.height_size,
                                                     self.stride, self.upsample_ratio, USE_CPU)

        total_keypoints_num = 0
        all_keypoints_by_type = []
        for keypoint_index in range(Pose.number_of_keypoints):  # 19th for bg
            total_keypoints_num += extract_keypoints(heatmaps[:, :, keypoint_index], all_keypoints_by_type,
                                                     total_keypoints_num)

        pose_entries, all_keypoints = group_keypoints(all_keypoints_by_type, pafs)

        for keypoint_id in range(all_keypoints.shape[0] ):
            all_keypoints[keypoint_id, 0] = (all_keypoints[keypoint_id, 0] * self.stride / self.upsample_ratio - pad[1] ) / scale
            all_keypoints[keypoint_id, 1] = (all_keypoints[keypoint_id, 1] * self.stride / self.upsample_ratio - pad[0] ) / scale

        current_poses = []
        for pose_entry in pose_entries:

            if len(pose_entry) == 0:
                continue

            pose_keypoints = np.full(shape=(Pose.number_of_keypoints, 2), fill_value=-1, dtype=np.int32)

            for keypoint_index in range(Pose.number_of_keypoints):
                if pose_entry[keypoint_index] == -1.0:  # No keypoint found:
                    continue

                pose_keypoints[keypoint_index, :] = tuple(int(keypoint) for keypoint in
                                                          all_keypoints[int(pose_entry[keypoint_index] ), :2] )

            current_poses.append(Pose(keypoints=pose_keypoints, confidence=pose_entry[18] ) )


        if self.track:
            track_poses(self.previous_poses, current_poses, smooth=self.smooth)
            self.previous_poses = current_poses

        return current_poses



def main (args=None):
    rclpy.init(args=args)

    image_repeater = Lop()
    rclpy.spin(image_repeater)
    image_repeater.destroy_node()

    rclpy.shutdown()



if __name__ == '__main__':
    main()
