
# Standard library:
import cv2
import numpy as np

# Local libraries:
from .keypoints import BODY_PARTS_KPT_IDS
from .one_euro_filter import OneEuroFilter

# ROS messages:
import lop2_interfaces.msg



class Pose:
    keypoint_names = ('nose',  'neck',
                      'r_sho', 'r_elb',  'r_wri', 'l_sho', 'l_elb',  'l_wri',
                      'r_hip', 'r_knee', 'r_ank', 'l_hip', 'l_knee', 'l_ank',
                      'r_eye', 'l_eye',
                      'r_ear', 'l_ear')
    number_of_keypoints = len(keypoint_names)
    sigmas = np.array(object=( .26, .79,
                               .79, .72, .62,  .79, .72, .62,
                              1.07, .87, .89, 1.07, .87, .89,
                               .25, .25,
                               .35, .35),
                      dtype=np.float32) / 10.0
    vars = (sigmas * 2) ** 2
    last_id = -1
    color_yellow = (0, 224, 255)


    def __init__ (self, keypoints: np.ndarray, confidence):
        super().__init__()

        self.keypoints = keypoints
        self.confidence = confidence
        self.bbox = Pose.get_bounding_box(self.keypoints)
        self.id = None
        self.filters = [OneEuroFilter() ] * Pose.number_of_keypoints


    @staticmethod
    def get_bounding_box (keypoints):
        found_keypoints = np.zeros(shape=(np.count_nonzero(keypoints[:, 0] != -1), 2),
                                   dtype=np.int32)
        found_keypoint_id = 0

        for keypoint_id in range(Pose.number_of_keypoints):
            if keypoints[keypoint_id, 0] == -1:
                continue

            found_keypoints[found_keypoint_id] = keypoints[keypoint_id]
            found_keypoint_id += 1

        bbox = cv2.boundingRect(found_keypoints)
        return bbox


    def update_id (self, id=None):
        self.id = id

        if self.id is None:
            self.id = Pose.last_id + 1
            Pose.last_id += 1


    def to_message (self) -> lop2_interfaces.msg.Pose2D:

        message = lop2_interfaces.msg.Pose2D()
        message.id = int(self.id)
        message.confidence = float(self.confidence)

       # Pose2D has a fixed size array of keypoints, appending does not work:
        for index, keypoint in enumerate(self.keypoints):
            pixel = message.keypoints[index]
            (pixel.x, pixel.y) = (int(dimension) for dimension in keypoint)  # Casting is required for Numpy array.


        return message



    def draw (self, image):
        assert self.keypoints.shape == (Pose.number_of_keypoints, 2)

        drawn_keypoints_indexes = list()

        # Exclude bodyparts ear-to-shoulder, the last two:
        for part_id in range(len(BODY_PARTS_KPT_IDS) - 2):
            keypoint_a_index = BODY_PARTS_KPT_IDS[part_id][0]
            keypoint_a_exists = (self.keypoints[keypoint_a_index, 0] != -1)

            if keypoint_a_exists and (keypoint_a_index not in drawn_keypoints_indexes):
                point_a = tuple(int(coordinate) for coordinate in self.keypoints[keypoint_a_index] )
                cv2.circle(img=image, center=point_a, radius=3, color=Pose.color_yellow, thickness=-1)
                drawn_keypoints_indexes.append(keypoint_a_index)


            keypoint_b_index = BODY_PARTS_KPT_IDS[part_id][1]
            keypoint_b_exists = (self.keypoints[keypoint_b_index, 0] != -1)

            if keypoint_b_exists and (keypoint_b_index not in drawn_keypoints_indexes):
                point_b = tuple(int(coordinate) for coordinate in self.keypoints[keypoint_b_index] )
                cv2.circle(img=image, center=point_b, radius=3, color=Pose.color_yellow, thickness=-1)
                drawn_keypoints_indexes.append(keypoint_b_index)


            if keypoint_a_exists and keypoint_b_exists:
                cv2.line(img=image, pt1=point_a, pt2=point_b, color=Pose.color_yellow, thickness=2)



def get_similarity (pose_a, pose_b, threshold=0.5):
    number_of_similar_keypoints = 0

    for keypoint_id in range(Pose.number_of_keypoints):
        if (pose_a.keypoints[keypoint_id, 0] == -1) and (pose_b.keypoints[keypoint_id, 0] == -1) :
            continue

        distance = np.sum( (pose_a.keypoints[keypoint_id] - pose_b.keypoints[keypoint_id] ) ** 2)
        area = max(pose_a.bbox[2] * pose_a.bbox[3], pose_b.bbox[2] * pose_b.bbox[3] )

        similarity = np.exp(-distance / (2 * (area + np.spacing(1) ) * Pose.vars[keypoint_id] ) )

        if similarity > threshold:
            number_of_similar_keypoints += 1

    return number_of_similar_keypoints


def track_poses (previous_poses, current_poses, threshold=3, smooth=False):
    '''Propagate poses ids from previous frame results. Id is propagated,
    if there are at least `threshold` similar keypoints between pose from previous frame and current.
    If correspondence between pose on previous and current frame was established, pose keypoints are smoothed.

    :param previous_poses: poses from previous frame with ids
    :param current_poses: poses from current frame to assign ids
    :param threshold: minimal number of similar keypoints between poses
    :param smooth: smooth pose keypoints between frames
    :return: None
    '''
    current_poses = sorted(current_poses, key=lambda pose: pose.confidence, reverse=True)  # match confident poses first
    mask = np.ones(shape=len(previous_poses), dtype=np.int32)

    for current_pose in current_poses:
        best_matched_id = None
        best_matched_pose_id = None
        best_matched_iou = 0

        for id, previous_pose in enumerate(previous_poses):
            if not mask[id]:
                continue

            iou = get_similarity(current_pose, previous_pose)
            if iou > best_matched_iou:
                best_matched_iou = iou
                best_matched_pose_id = previous_pose.id
                best_matched_id = id

        if best_matched_iou >= threshold:
            mask[best_matched_id] = 0
        else:  # pose not similar to any previous
            best_matched_pose_id = None

        current_pose.update_id(best_matched_pose_id)

        if smooth:
            for keypoint_id in range(Pose.number_of_keypoints):
                if current_pose.keypoints[keypoint_id, 0] == -1:
                    continue

                # Reuse filter if previous pose has valid filter:
                if (best_matched_pose_id is not None) and (previous_poses[best_matched_id].keypoints[keypoint_id, 0] != -1):
                    current_pose.filters[keypoint_id] = previous_poses[best_matched_id].filters[keypoint_id]

                current_pose.keypoints[keypoint_id] = current_pose.filters[keypoint_id](current_pose.keypoints[keypoint_id] )

            current_pose.bbox = Pose.get_bounding_box(current_pose.keypoints)
