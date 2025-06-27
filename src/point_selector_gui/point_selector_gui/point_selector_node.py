import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2 as cv

QUEUE_SIZE = 10

class PointSelector(Node):

    def __init__(self):
        super().__init__('point_selector_node')

        # create subscriptions to topics that realsense2 camera node 
        # publishes to
        self.color_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            QUEUE_SIZE
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_callback,
            QUEUE_SIZE
        )
        self.camera_intrinsics_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.info_callback,
            QUEUE_SIZE
        )

        self.color_image = None
        self.depth_image = None
        self.camera_intrinsics = None

def main() -> None:
    print("Hello from point selector Node!")
    return

if __name__ == '__main__':
    main()