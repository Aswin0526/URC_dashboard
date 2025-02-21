import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from threading import Thread, Lock
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, MagneticField, Imu

zed_img_topic ="/zed/zed_node/left/image_rect_color"
zed_depth_topic ="/zed/zed_node/depth/depth_registered"
zed_mag_topic = "/zed/zed_node/imu/mag"
zed_imu_topic = "/zed/zed_node/imu/data"


def quaternion_to_euler(quaternion):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw)
    """
    x, y, z, w = quaternion

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    yaw = math.degrees(yaw)


    # Convert radians to degrees
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)

    return roll, pitch, yaw

def calculate_angle_turned(initial_yaw,yaw):
        # Convert quaternion to Euler angles
        

        if initial_yaw is None:
            # Set initial yaw
            initial_yaw = yaw
            print(f"Initial yaw set to: {yaw:.2f}°")
            return 0.0  # No turn at initialization

        # Calculate the relative angle turned
        turned_angle = yaw - initial_yaw

        # Normalize to range [-180°, 180°]
        turned_angle = (turned_angle + 180) % 360 - 180

        print(f"Current yaw: {yaw:.2f}°, Turned angle: {turned_angle:.2f}°")
        return turned_angle



class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")

        self.stored_yaw = False
        self.start_yaw = None
        self.image = np.zeros((720, 1280, 3))
        self.depth_img = np.full((720, 1280), np.inf)
        self.mag: MagneticField = None 
        self.lock = Lock()

        self.bridge = CvBridge()
        self.image_subscriber = self.create_subscription(Image, zed_img_topic, self.image_callback, 10)
        self.depth_subscriber = self.create_subscription(Image, zed_depth_topic, self.depth_callback, 10)
        self.mag_subscriber = self.create_subscription(MagneticField, zed_mag_topic, self.mag_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, zed_imu_topic, self.imu_callback, 10)

    def image_callback(self, msg):
        with self.lock:
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def depth_callback(self, msg):
        with self.lock:
            try:
                self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            except Exception as e:
                self.get_logger().error(f"Error processing depth image: {e}")
    
    def mag_callback(self, msg):
        with self.lock:
            self.mag = msg
            x = msg.magnetic_field.x
            y = msg.magnetic_field.y
            z = msg.magnetic_field.z

            theta = math.atan2(x, y) * (180 * (7 / 22))
            # print("angle: ", theta)
            # time.sleep(1)

    def imu_callback(self, msg):
        with self.lock:
            self.imu = msg

            orientation = self.imu.orientation
            x = orientation.x
            y = orientation.y
            z = orientation.z
            w = orientation.w

            roll,pitch,yaw = quaternion_to_euler([x,y,z,w])


            if not self.stored_yaw:
                self.start_yaw = yaw
                self.stored_yaw = True

            # print(f"x: {x}, y: {y}, z: {z}, w: {w}")
            deviation = calculate_angle_turned(self.start_yaw, yaw)

    
            print(f"w: {w}, roll: {roll}, pitch:{pitch}, yaw:{yaw}, deviation:{deviation}")
            time.sleep(1)


    def get_latest_data(self):
        with self.lock:
            return self.image, self.depth_img


if __name__ == "__main__":
    rclpy.init()
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

