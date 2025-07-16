import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image, CameraInfo
# from sensor.msgs.msg import PointCloud2
from welding_path_interfaces.srv import PixelPath
from welding_path_interfaces.msg import Pixel

from message_filters import Subscriber, ApproximateTimeSynchronizer

from cv_bridge import CvBridge
import cv2 as cv

import numpy as np

from rclpy.qos import qos_profile_sensor_data

QUEUE_SIZE = 10

class PointSelector(Node):

    def __init__(self):
        super().__init__('point_selector_node')
        self.get_logger().info("Point Selector Node started successfully.")

        self.declare_parameter('num_clicked_points', 2)  # default = 2

        # Initialise client for pixel_path service
        self.cli = self.create_client(
            PixelPath,
            'pixel_path'
        )
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PixelPath.Request()

        # Subscribes to rgb image for opencv gui display
        self.gui_display_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.gui_display_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.color_image = None

        # Initialise OpenCv GUI
        self.bridge = CvBridge()
        cv.namedWindow('RGB Stream', cv.WINDOW_NORMAL)
        cv.setMouseCallback('RGB Stream', self.mouse_callback)
        
        self.clicked_points = []    # array of clicked points [[u, v], ...]
    
    def send_request(self, clicked_points):
        """
        Sends a asynchronous service request with the provided clicked 2D 
        points.

        Args:
            clicked_points: A list containing num_clicked_points 2D pixel
                            coordinates.
        
        Returns:
            rclpy.task.Future: A future object that will contain the response
                                once the service call completes.
        
        """
        self.req.clicked_points = clicked_points
        return self.cli.call_async(self.req)

    def gui_display_callback(self, msg: Image) -> None:
        """
        Callback function that displays RGB image in an OpenCV Window 'RGB Stream'.
        
        Function is triggered when gui_display_sub subscriber recieves data.

        Args:
            msg: The RGB image message.
        
        Returns None
        """
        self.color_image = msg

        # convert ros img format to opencv format (numpy array)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv.imshow('RGB Stream', frame)
        cv.waitKey(1)
    
    def mouse_callback(self,
                       event: int, 
                       u: int, 
                       v: int, 
                       flags: int, 
                       param) -> None:
        """
        Callback function that processing mouse-clicks on the OpenCv
        'RGB Stream' window.
        
        Function is triggered when a mouse-click is detected on the window.
        It records the pixel coordinates of button clicks.

        Args:
            event: OpenCV event type (e.g. cv.EVENT_LBUTTONDOWN)
            u: The x-coordinate (col) of mouse pointer
            v: The y-coordinate (row) of mouse pointer
            flags: Any relevant flags passed by OpenCV
            param: Extra parameters supplied by OpenCV
        
        Returns:
            None
        """
        if (event == cv.EVENT_LBUTTONDOWN):
            self.get_logger().info(f"User clicked at pixel ({u}, {v})")
            self.clicked_points.append(Pixel(x=u, y=v))

            if (
                len(self.clicked_points) >= self.get_parameter('num_clicked_points').value
            ):
                self.get_logger().info("Number of clicks required has been reached. \nRequesting Path Planning.")

                # destroy window and disable callbacks
                cv.destroyWindow('RGB Stream')
                cv.setMouseCallback('RGB Stream', lambda *args : None)

                # send request to path_planner node
                self.send_request(self.clicked_points)
    

def main(args=None) -> None:
    rclpy.init(args=args)
    pointSelector = PointSelector()

    rclpy.spin(pointSelector)

    pointSelector.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()