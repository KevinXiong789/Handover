import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading

from flask import Flask, Response

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_stream_node')
        self.bridge = CvBridge()

        # Initialize Flask app
        self.app = Flask(__name__)

        @self.app.route('/stream')
        def stream():
            return Response(self.camera_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

        # Define your camera topic names
        self.camera_topics = ['/lop_recCamera/pose_image', '/lop_recCameraL/pose_image', '/lop_recCameraR/pose_image']
        self.image_subscribers = []

        for topic in self.camera_topics:
            self.image_subscribers.append(self.create_subscription(Image, topic, self.camera_callback, 10))

        self.cameras = [None] * len(self.camera_topics)
        self.frames = [None] * len(self.camera_topics)

    def camera_stream_generator(self, camera_index):
        while True:
            if self.frames[camera_index] is not None:
                encoded_frame = cv2.imencode('.jpg', self.frames[camera_index])[1].tobytes()
                yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + encoded_frame + b'\r\n')

    def camera_stream(self):
        for camera_index in range(len(self.cameras)):
            yield from self.camera_stream_generator(camera_index)

    def camera_callback(self, msg, camera_index):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.frames[camera_index] = cv_image

    def main(self):
        thread = threading.Thread(target=self.node_thread)
        thread.start()
        self.app.run(host='0.0.0.0', port=5000)

    def node_thread(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamNode()
    node.main()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
