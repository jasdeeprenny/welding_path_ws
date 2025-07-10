import rclpy
from rclpy.node import Node
from welding_path_interfaces.srv import PixelPath
from sensor_msgs.msg import PointCloud2

from sensor_msgs_py import point_cloud2
import numpy as np
from skimage.draw import line

from rclpy.qos import qos_profile_sensor_data

import open3d as o3d

from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from tf_transformations import quaternion_from_matrix

class PathPlannerService(Node):

    def __init__(self):
        super().__init__('path_planner_node')
        print("Hello from pathPlanner Node")

        # Initialise service to recieve path planning requests
        self.srv = self.create_service(
            PixelPath,
            'pixel_path',
            self.pixel_path_callback
        )

        # Initialise subscriber to point cloud from realsense2_camera node
        self.point_cloud_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.point_cloud_callback,
            qos_profile=qos_profile_sensor_data
        )

        # Initialise pose array publisher
        # self.pose_array_pub = self.create_publisher(PoseArray, 'pose_array_path', 10)
        self.point_cloud_msg = None
        self.point_cloud_array = None
    
    def calculate_2d_path(self, pixel_start, pixel_end, num_points=30):
        """
        Interpolates a line of pixel coordinates between two points (inclusive).

        Args:
            pixel_start (tuple): Starting pixel coordinate (x, y).
            pixel_end (tuple): Ending pixel coordinate (x, y).
            num_points (int): Number of points to interpolate along the line.

        Returns:
            List[Tuple[int, int]]: List of integer pixel coordinates.
        """

        x_vals = np.linspace(pixel_start[0], pixel_end[0], num_points)
        y_vals = np.linspace(pixel_start[1], pixel_end[1], num_points)

        pixel_points = [(int(round(x)), int(round(y))) for x, y in zip(x_vals, y_vals)]

        seen = set()
        unique_pixels = []
        for point in pixel_points:
            if point not in seen:
                seen.add(point)
                unique_pixels.append(point)

        return unique_pixels
        
    def pixel_path_callback(self, request, response):
        """
        Callback service function to process a request for computing a 3D
        path between two 2D pixel coordinates selected in an image frame.

        The function computes a 2D pixel path between the given start and end
        pixels and then projects this 2D path into a 3D space using point cloud
        data.

        Args:
            request: The service request object containing:
                        - pixel_start (Tuple[int, int]): Start pixel coordinate.
                        - pixel_end (Tuple[int, int]): End pixel coordinate.
            response: The service response object to populate.
        
        Returns:
            response: The service response object indicating success and
                        feedback to client.
        """

        print("====Request Recieved!====")
        print(f"Point A: {request.pixel_start} -> Point B: {request.pixel_end}")
    
        # path planning
        pixel_path = self.calculate_2d_path(request.pixel_start, request.pixel_end)
        print("___2D Linear Path___")
        print(pixel_path)

        point_path = self.calculate_3d_path(pixel_path)

        print("___3D Linear Path___")
        print(point_path)

        filtered_point_path = self.filter_point_path(point_path, pixel_path)
        print("___3D Filtered Point Path___")
        print(filtered_point_path)

        self.detect_invalid_path(point_path)

        # find poses of each point on 3d path
        pose_array = self.generate_pose_array(point_path)

        # publish as PoseArray to visualise in Rviz2

        # response?
        # if success -> response: response.success = True ...
        response.success = True
        response.message = "====Recieved 2x 2D pixel coordinates===="
        return response

    def filter_point_path(self, point_path, pixel_path):
        """
        Filters out invalid (0, 0, 0) points in a 3D path by checking
        3D projections of neighbouring pixels in the point cloud.

        Args:
            point_path (List[Tuple[int, int, int]]): 3D path with possibly
                invalid points.
            pixel_path (List[Tuple[int, int]]): Corresponding 2D pixel path.
        
        Returns:
            List[Tuple[int, int, int]]: A filtered 3D path with no invalid points.

        """
        offsets = [(-1, -1), (-1, 0), (-1, 1),
                   (0, -1),           ( 0, 1),
                   ( 1, -1), ( 1, 0), ( 1, 1)]
        
        height, width, _ = self.point_cloud_array.shape

        for i in range(len(point_path)):

            x, y, z = point_path[i]
            if (x == 0) and (y == 0) and (z == 0):
                print(f"Invalid point at index {i}")
                replaced = False

                col, row = pixel_path[i]
                for dcol, drow in offsets:
                    print(f"Searching dcol: {dcol} drow: {drow}")
                    new_row = row + drow
                    new_col = col + dcol

                    if (0 <= new_row < height) and (0 <= new_col < width):
                        new_point = tuple(self.point_cloud_array[new_row, new_col])
                        new_x, new_y, new_z = new_point
                    
                        if (new_point != (0, 0, 0)):
                            point_path[i] = new_point
                            replaced = True
                            break

                if not replaced:    # how to handle this?
                    print(f"Failed to replace point at index {i}")
        
        return point_path

    def detect_invalid_path(self, point_path, jump_threshold=0.05) -> bool:
        """
        Detects large spatial gaps between consecutive 3D points in a 3D path
        considering euclidean distance.

        Args:
            point_path: List of 3D points (x, y, z) of path
            jump_threshold: Maximum allowed euclidean distance between
                            consecutive valid points.
        
        Returns:
            True if all valid jumps are within threshold; False otherwise.
        """
        for i in range(len(point_path) - 1):
            curr_point, next_point = point_path[i], point_path[i+1]

            if (curr_point == (0, 0, 0)) or (next_point == (0, 0, 0)):
                continue
            
            euclidean_dist = np.linalg.norm(np.subtract(next_point, curr_point))            
            if (euclidean_dist > jump_threshold):
                print("!Jump Threshold Exceeded!")
                print(f"Distance: {euclidean_dist} between points: {i} and {i+1}")

                return False
        return True

    def compute_tangent_vectors(self, point_path):
        """
        Computes the normalised tangent (direction) vectors at each point on 
        the 3D path.

        Args:
            point_path (List[Tuple[int, int, int]]): List of 3D coordinates.
        
        Returns:
            List[np.ndarray]: Tangent vector at each point.
        """

        tangents = []

        for i in range(len(point_path)):
            curr_point = np.array(point_path[i])

            if (i == 0):
                next_point = np.array(point_path[i+1])
                diff = next_point - curr_point

            elif (i == len(point_path) - 1):
                prev_point = np.array(point_path[i-1])
                diff = curr_point - prev_point
            
            else:
                prev_point = np.array(point_path[i-1])
                next_point = np.array(point_path[i+1])
                diff = next_point - prev_point
            
            norm = np.linalg.norm(diff)

            if (norm == 0):     # prev_point and next_point identical
                tangent = None
            else:
                tangent = diff / norm   # normalise vector

            tangents.append(tangent)
        
        return tangents

    def compute_normal_vectors(self, point_path, radius=0.02, max_nn=30):
        """
        Computes surface normals at each 3D point in the path by referencing
        the full point cloud.

        Args:
            point_path (List[Tuple[float, float, float]]): The 3D coordinates
                                                            of the path.
            radius (float): Radius for normal estimation search.
            max_nn (int): Maximum number of neighbours for normal estimation.
        
        Returns:
            List[np.ndarray or None]: Estimated normals for each 3D point in 
                                        point_path; None if point is invalid.

        """

        open3d_point_cloud = self.pointcloud2_to_open3d()

        # compute surface normals for each point in full cloud
        open3d_point_cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=radius, 
                max_nn=max_nn
            )
        )
        # ensures all normals point in the same hemisphere
        open3d_point_cloud.orient_normals_consistent_tangent_plane(k=max_nn)

        kdtree = o3d.geometry.KDTreeFlann(open3d_point_cloud)

        normals = []
        for point in point_path:
            if (point == (0, 0, 0)):    # skip invalid points
                normals.append(None)
                continue
            
            point_np = np.array(point)

            # finds index of closest point in full cloud to point_np
            [_, idxs, _] = kdtree.search_knn_vector_3d(point_np, 1)

            if (idxs):
                normal = np.asarray(open3d_point_cloud.normals)[idxs[0]]
                normals.append(normal)
            else:
                normals.append(None)
        
        return normals

    def pointcloud2_to_open3d(self):
        """
        Converts a ROS2 sensor_msgs/msg/PointCloud2 message into an Open3d
        compatible PointCloud object.

        Returns:
            open3d.geometry.PointCloud: The point cloud constructed from the 
                                        valid 3D points.
        """
        points = point_cloud2.read_points(
            self.point_cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )
        xyz = np.array(list(points))
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        return pcd

    def generate_pose_array(self, point_path):

        tangents = self.compute_tangent_vectors(point_path)
        normals = self.compute_normal_vectors(point_path)

        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now.to_msg()

        for i, (position, x_axis, z_axis) in enumerate(zip(point_path, tangents, normals)):
            if (position == (0, 0, 0)) or (x_axis is None) or (z_axis is None):
                continue

            # compute y-axis via cross product
            y_axis = np.cross(z_axis, x_axis)
            if (np.linalg.norm(y_axis) == 0):
                continue
            y_axis /= np.linalg.norm(y_axis)

            # re-orthohonalise axes
            x_axis = np.cross(y_axis, z_axis)
            x_axis /= np.linalg.norm(x_axis)
            z_axis /= np.linalg.norm(z-axis)

            # build rotation matrix
            rot_matrix = np.eye(4)
            rot_matrix[0:3, 0] = x_axis
            rot_matrix[0:3, 1] = y_axis
            rot_matrix[0:3, 2] = z_axis

            quat = quaternion_from_matrix(rot_matrix)

            pose = Pose()
            pose.position = Point(*position)
            pose.orienation = Quaternion(*quat)

            pose_array.poses.append(pose)
        
        return pose_array

    def convert_pointcloud2_to_xyz_arr(self):
        """
        Converts a sensor_msgs/msg/PointCloud2 message to a NumPy array of
        shape (height, width, 3) representing the 3D coordinates of each pixel.

        Returns:
            3D Numpy Array where xyz.shape == (height, width, 3), and a pixel
            (col, row) can be mapped to 3D coordinates by: (x, y, z) = xyz[row, col]
        """

        # extract 2D structure of the point cloud
        width = self.point_cloud_msg.width
        height = self.point_cloud_msg.height

        # create 1D structured array of shape (H * W) with fields x, y, z
        points = np.fromiter(
            point_cloud2.read_points(
                self.point_cloud_msg,
                field_names=("x", "y", "z"),
                skip_nans=False
            ),
            dtype=[
                ("x", np.float32),
                ("y", np.float32),
                ("z", np.float32)
            ],
            count=width*height
        )

        # stack into 2D array of shape (H*W, 3): one row per point, 
        # 3 values (x, y, z) per row
        xyz = np.stack((points['x'], points['y'], points['z']), axis=-1)

        # reshape 2D array to 3D matrix (H, W, 3)
        xyz = xyz.reshape((height, width , 3))
        return xyz

    def calculate_3d_path(self, linear_2d_path):
        """
        Projects a 2D pixel path into 3D space using the current PointCloud2
        data.

        For each (col, row) pixel coordinate in the linear_2d_path, the 
        corresponding (x, y, z) 3D point is retrieved from the point cloud.

        Args:
            linear_2d_path (List[Tuple[int, int]]): A list of 2D pixel
                                                    coordinates (col, row)
        
        Returns: 
            List[Tuple[int, int, int]]: A list of 3D coordinates (x, y, z)
                corresponding to the input pixel path w.r.t the camera_depth_
                optical_frame
        """

        # reformat PointCloud2 msg format to pixel indexable 2d image array
        self.point_cloud_array = self.convert_pointcloud2_to_xyz_arr()

        linear_3d_coordinates = []
        for point in linear_2d_path:
            col, row = point

            linear_3d_coordinates.append(tuple(self.point_cloud_array[row, col]))

        return linear_3d_coordinates

    def point_cloud_callback(self, point_cloud_msg) -> None:
        # print("Recieved point_cloud_msg!")

        self.point_cloud_msg = point_cloud_msg
        return None
    
def main(args=None) -> None:
    rclpy.init(args=args)

    pathPlanner = PathPlannerService()
    rclpy.spin(pathPlanner)

    pathPlanner.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()