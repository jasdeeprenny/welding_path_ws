import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from sensor.msgs.msg import PointCloud2

from message_filters import Subscriber, ApproximateTimeSynchronizer

from cv_bridge import CvBridge
import cv2 as cv

import numpy as np
import pyrealsense2 as rs

QUEUE_SIZE = 10
NUM_CLICKS = 2

class PointSelector(Node):

    def __init__(self):
        super().__init__('point_selector_node')

        # Subscribes to rgb image for opencv gui display
        self.gui_display_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.gui_display_callback,
            QUEUE_SIZE
        )

        # Subscribes to aligned point cloud
        # self.point_cloud_sub = self.create_subscription(
        #     PointCloud2,
        #     '/camera/depth/color/points',
        #     self.point_cloud_callback,
        #     QUEUE_SIZE
        # )

        # Synchronised subscribers
        self.rgb_sub = Subscriber(
            self, 
            Image, 
            '/camera/camera/color/image_raw'
        )
        self.depth_sub = Subscriber(
            self, 
            Image, 
            '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.camera_intrinsics_sub = Subscriber(
            self, 
            CameraInfo,
            '/camera/camera/color/camera_info'
        )

        self.approx_sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.camera_intrinsics_sub],
            queue_size=15,
            slop=0.05           # 50ms delta allowed
        )
        self.approx_sync.registerCallback(self.synced_callback)

        self.color_image = None
        self.depth_image = None
        self.camera_intrinsics = None
        self.rs_intrinsics = None

        # Initialise OpenCv GUI
        self.bridge = CvBridge()
        cv.namedWindow('RGB Stream', cv.WINDOW_NORMAL)
        cv.setMouseCallback('RGB Stream', self.mouse_callback)
        
        self.clicked_points = []    # array of clicked points [(u, v), ...]
        self.clicked_points_3D = [] # array of 3D clicked coordinates [(x, y, z) ...]
    
    # def point_cloud_callback(self, point_cloud_msg: PointCloud2) -> None:
    #     return
    #     print("Point Cloud Recieved!")

    def gui_display_callback(self, msg: Image) -> None:
        """
        Callback function that displays RGB image in an OpenCV Window 'RGB Stream'.
        
        Function is triggered when gui_display_sub subscriber recieves data.

        Args:
            msg: The RGB image message.
        
        Returns None
        """
        
        #print("Colour info recieved!")

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
            print(f"Mouse clicked at ({u}, {v})")

            self.clicked_points.append((u, v))

            if (len(self.clicked_points) >= NUM_CLICKS):
                print("Max num clicks reached")

                # destroy window and disable callbacks
                cv.destroyWindow('RGB Stream')
                cv.setMouseCallback('RGB Stream', lambda *args : None)
    
    def synced_callback(self, 
                        rgb_msg: Image, 
                        depth_msg: Image,
                        camera_info_msg: CameraInfo) -> None:
        """
        Synchronised callback function for processing RGB image, depth image,
        and camera intrinsics.

        Function is triggered when synchronised messages from the RGB image,
        depth image, and camera intrinsics are recieved within a slop time.
        
        Args:
            rgb_msg: The RGB image message
            depth_msg: The aligned depth image message
            camera_info_msg: The intrinsic parameters of the RGB camera.
        
        Returns None
        """
        if (len(self.clicked_points) < NUM_CLICKS):
            return
        
        # calculate 3D coordinate of clicked points if all required points
        # have been selected

        # convert ros image msgs to numpy arr format
        color_np = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
        depth_np = ( self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
                    .astype(np.float32) * 0.001)

        print("RGB shape: ", color_np.shape)
        print("Depth shape: ", depth_np.shape)

        # build pyrealsense2 intrinsics object 
        if self.rs_intrinsics is None:
            self.rs_intrinsics = self.camera_info_to_rs_intrinsics(
                                    camera_info_msg)
        
        self.clicked_points_3D = []
        for (u, v) in self.clicked_points:  # (col, row) = (x,y)

            # bounds check (if user clicks outside of image)
            if ((v < 0) or (v >= depth_np.shape[0]) 
                or (u < 0) or (u >= depth_np.shape[1])):
                # error handling
                print("click outside frame bounds")
                continue
            
            # bounds check (if user's click is too far away)
            depth_value = depth_np[v, u]
            if (depth_value <= 0) or (np.isnan(depth_value)):
                # error handling
                print("click outside depth bounds")
                continue
            
            xyz = rs.rs2_deproject_pixel_to_point(self.rs_intrinsics, 
                                                [u, v], 
                                                depth_value
                )
            
            self.clicked_points_3D.append(tuple(xyz))

        # now we plan path between 3d clicked points
        print("Ready to Path Plan!")

        print("\n==== Pixel to 3D Point Conversions ====")
        for i, ((u, v), (x, y, z)) in enumerate(zip(self.clicked_points, self.clicked_points_3D)):
            print(f"[{i+1}] Pixel (u={u}, v={v}) → 3D Point (x={x:.3f}, y={y:.3f}, z={z:.3f}) [m]")
        print("========================================\n")

    
    def camera_info_to_rs_intrinsics(self, camera_info_msg: CameraInfo) -> rs.intrinsics:
        """
        Helper method that converts ROS2 sensor_msgs/CameraInfo to pyrealsense2
        intrinsics.

        Args:
            camera_info_msg: ROS2 message containing intrinsic parameters of the
                                camera (projection matrix, resolution etc.)

        Returns:
            rs.intrinsics: A pyrealsense2 intrinsics object populated with the 
                            camera parameters gained from camera_info_msg
        """
        intrinsics = rs.intrinsics()

        # set image resolution
        intrinsics.width = camera_info_msg.width
        intrinsics.height = camera_info_msg.height

        # extract intrinsic camera matrix K
        intrinsics.fx = camera_info_msg.k[0]    # fx
        intrinsics.fy = camera_info_msg.k[4]    # fy
        intrinsics.ppx = camera_info_msg.k[2]   # cx
        intrinsics.ppy = camera_info_msg.k[5]   # cy

        # set distortion model to none
        intrinsics.model = rs.distortion.none
        intrinsics.coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]

        return intrinsics
    
    def project_pixel_to_point(self, u, v, depth_image, K):
        """
        Raw mathematical implementation of de-projecting pixels to 3d points.
        
        Currently not used (implemented through pyrealsense2 library)
        """
        # get depth at pixel (u, v)
        Z = depth_image[v, u]

        # handle invalid depth
        if (Z == 0):
            print(f"Point ({u}, {v}) is invalid!")
        
        # get camera intrinsics
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        # apply projection formulae
        X = (u-cx) * Z / fx
        Y = (v-cy) * Z / fy

        print(f"3D Coordinate: ({X}, {Y}, {Z})")
        return (X, Y, Z)


def main(args=None) -> None:
    rclpy.init(args=args)   # init ros2 python client lib
    print("Hello from point selector main!")

    pointSelector = PointSelector()

    rclpy.spin(pointSelector)   # keeps node alive and handles callbacks

    pointSelector.destroy_node()
    rclpy.shutdown()
    return

if __name__ == '__main__':
    main()