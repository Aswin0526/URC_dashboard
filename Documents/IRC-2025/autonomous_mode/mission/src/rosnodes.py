import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from threading import Thread, Lock

zed_img_topic ="/zed/zed_node/left/image_rect_color"
zed_depth_topic ="/zed/zed_node/depth/depth_registered"

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.image = np.zeros((720, 1280, 3))
        self.depth_img = np.full((720, 1280), np.inf)
        self.lock = Lock()

        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, zed_img_topic, self.image_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, zed_depth_topic, self.depth_callback, 10)

    def image_callback(self, msg):
        with self.lock:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_callback(self, msg):
        with self.lock:
            try:
                self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            except Exception as e:
                self.get_logger().error(f"Error processing depth image: {e}")

    def get_latest_data(self):
        with self.lock:
            return self.image, self.depth_img

def _spinner(node):
    rclpy.spin(node)

def start_node():
    rclpy.init()
    node = ImageSubscriber()
    thread_obj = Thread(target=_spinner, args=(node,))
    thread_obj.start()
    return node