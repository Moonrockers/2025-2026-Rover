#!/usr/bin/env python3
"""
Custom SLAM Implementation from Scratch for Intel RealSense D435i
This implements visual odometry, mapping, and localization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import numpy as np
import cv2
from collections import deque
from scipy.spatial.transform import Rotation as R
import struct

class FeatureTracker:
    """Tracks ORB features between consecutive frames"""
    
    def __init__(self, n_features=2000):
        self.orb = cv2.ORB_create(nfeatures=n_features)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_frame = None
        
    def detect_and_compute(self, frame):
        """Detect ORB features in frame"""
        keypoints, descriptors = self.orb.detectAndCompute(frame, None)
        return keypoints, descriptors
    
    def match_features(self, desc1, desc2):
        """Match features using ratio test"""
        if desc1 is None or desc2 is None:
            return []
        
        matches = self.matcher.knnMatch(desc1, desc2, k=2)
        
        # Lowe's ratio test
        good_matches = []
        for match_pair in matches:
            if len(match_pair) == 2:
                m, n = match_pair
                if m.distance < 0.75 * n.distance:
                    good_matches.append(m)
        
        return good_matches


class PoseEstimator:
    """Estimates camera pose from matched features"""
    
    def __init__(self, camera_matrix, baseline=0.05):
        self.camera_matrix = camera_matrix
        self.baseline = baseline
        
    def estimate_pose(self, kp1, kp2, matches, depth1):
        """Estimate relative pose using PnP"""
        if len(matches) < 10:
            return None, None
        
        # Extract matched points
        pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
        pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
        
        # Get 3D points from depth
        points_3d = []
        points_2d = []
        
        for i, (pt1, pt2) in enumerate(zip(pts1, pts2)):
            x, y = int(pt1[0]), int(pt1[1])
            if 0 <= y < depth1.shape[0] and 0 <= x < depth1.shape[1]:
                z = depth1[y, x]
                if z > 0 and z < 10.0:  # Valid depth between 0 and 10 meters
                    # Back-project to 3D
                    fx = self.camera_matrix[0, 0]
                    fy = self.camera_matrix[1, 1]
                    cx = self.camera_matrix[0, 2]
                    cy = self.camera_matrix[1, 2]
                    
                    X = (x - cx) * z / fx
                    Y = (y - cy) * z / fy
                    Z = z
                    
                    points_3d.append([X, Y, Z])
                    points_2d.append(pt2)
        
        if len(points_3d) < 10:
            return None, None
        
        points_3d = np.array(points_3d, dtype=np.float32)
        points_2d = np.array(points_2d, dtype=np.float32)
        
        # Solve PnP with RANSAC
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            points_3d, points_2d, self.camera_matrix, None,
            iterationsCount=100,
            reprojectionError=8.0,
            confidence=0.99
        )
        
        if not success or inliers is None or len(inliers) < 10:
            return None, None
        
        return rvec, tvec


class OccupancyGridMapper:
    """Creates 2D occupancy grid map"""
    
    def __init__(self, resolution=0.05, width=400, height=400):
        self.resolution = resolution  # meters per cell
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=np.int8)
        self.origin_x = width // 2
        self.origin_y = height // 2
        
    def world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates"""
        grid_x = int(x / self.resolution) + self.origin_x
        grid_y = int(-y / self.resolution) + self.origin_y  # Negative because grid Y is inverted
        return grid_x, grid_y
    
    def update_from_depth(self, depth_image, camera_matrix, pose):
        """Update occupancy grid from depth image and camera pose"""
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        # Extract rotation and translation from pose
        rotation_matrix = pose[:3, :3]
        translation = pose[:3, 3]
        
        # Downsample depth image for efficiency
        step = 10
        height, width = depth_image.shape
        
        for v in range(0, height, step):
            for u in range(0, width, step):
                z = depth_image[v, u]
                
                if z > 0.1 and z < 8.0:  # Valid depth range
                    # Back-project to camera frame
                    x_cam = (u - cx) * z / fx
                    y_cam = (v - cy) * z / fy
                    z_cam = z
                    
                    point_cam = np.array([x_cam, y_cam, z_cam])
                    
                    # Transform to world frame
                    point_world = rotation_matrix @ point_cam + translation
                    
                    # Update grid
                    grid_x, grid_y = self.world_to_grid(point_world[0], point_world[2])
                    
                    if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
                        # Mark as occupied
                        self.grid[grid_y, grid_x] = min(100, self.grid[grid_y, grid_x] + 10)
    
    def get_occupancy_grid_msg(self, frame_id='map', stamp=None):
        """Convert to ROS OccupancyGrid message"""
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp if stamp else rclpy.time.Time().to_msg()
        
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = -self.origin_x * self.resolution
        msg.info.origin.position.y = -self.origin_y * self.resolution
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Flatten and convert
        msg.data = self.grid.flatten().tolist()
        
        return msg


class CustomSLAM(Node):
    """Main SLAM node implementing visual odometry and mapping"""
    
    def __init__(self):
        super().__init__('custom_slam')
        
        # Parameters
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('max_depth', 8.0)
        
        # State
        self.bridge = CvBridge()
        self.feature_tracker = FeatureTracker()
        self.pose_estimator = None  # Initialize after camera info
        self.mapper = OccupancyGridMapper(
            resolution=self.get_parameter('map_resolution').value
        )
        
        # Camera calibration
        self.camera_matrix = None
        self.camera_info_received = False
        
        # Current state
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.trajectory = []
        
        # Frame storage
        self.prev_rgb = None
        self.prev_depth = None
        self.current_rgb = None
        self.current_depth = None
        self.frame_count = 0
        
        # IMU integration (optional, for better accuracy)
        self.imu_data = deque(maxlen=100)
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', 
            self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.camera_info_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/camera/camera/imu',
            self.imu_callback, 10
        )
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/slam/odometry', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/slam/map', 10)
        self.path_pub = self.create_publisher(Path, '/slam/trajectory', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/slam/pointcloud', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timers
        self.create_timer(0.1, self.process_frames)
        self.create_timer(1.0, self.publish_map)
        
        self.get_logger().info('Custom SLAM initialized - waiting for camera info...')
    
    def camera_info_callback(self, msg):
        """Initialize camera matrix from camera info"""
        if not self.camera_info_received:
            self.camera_matrix = np.array([
                [msg.k[0], 0, msg.k[2]],
                [0, msg.k[4], msg.k[5]],
                [0, 0, 1]
            ])
            self.pose_estimator = PoseEstimator(self.camera_matrix)
            self.camera_info_received = True
            self.get_logger().info(f'Camera calibrated: fx={msg.k[0]:.1f}, fy={msg.k[4]:.1f}')
    
    def rgb_callback(self, msg):
        """Store latest RGB frame"""
        self.current_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def depth_callback(self, msg):
        """Store latest depth frame"""
        # RealSense depth is in millimeters, convert to meters
        depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.current_depth = depth_mm.astype(np.float32) / 1000.0
    
    def imu_callback(self, msg):
        """Store IMU data for future use"""
        self.imu_data.append(msg)
    
    def process_frames(self):
        """Main processing loop for SLAM"""
        if not self.camera_info_received:
            return
        
        if self.current_rgb is None or self.current_depth is None:
            return
        
        self.frame_count += 1
        
        # Convert to grayscale for feature detection
        gray = cv2.cvtColor(self.current_rgb, cv2.COLOR_BGR2GRAY)
        
        # Detect features
        keypoints, descriptors = self.feature_tracker.detect_and_compute(gray)
        
        if self.prev_rgb is not None:
            # Match features with previous frame
            prev_gray = cv2.cvtColor(self.prev_rgb, cv2.COLOR_BGR2GRAY)
            prev_kp, prev_desc = self.feature_tracker.prev_keypoints, self.feature_tracker.prev_descriptors
            
            if prev_desc is not None and descriptors is not None:
                matches = self.feature_tracker.match_features(prev_desc, descriptors)
                
                if len(matches) >= 10:
                    # Estimate pose change
                    rvec, tvec = self.pose_estimator.estimate_pose(
                        prev_kp, keypoints, matches, self.prev_depth
                    )
                    
                    if rvec is not None and tvec is not None:
                        # Convert to transformation matrix
                        R_mat, _ = cv2.Rodrigues(rvec)
                        T_delta = np.eye(4)
                        T_delta[:3, :3] = R_mat
                        T_delta[:3, 3] = tvec.flatten()
                        
                        # Update current pose
                        self.current_pose = self.current_pose @ np.linalg.inv(T_delta)
                        
                        # Add to trajectory
                        position = self.current_pose[:3, 3]
                        self.trajectory.append(position.copy())
                        
                        # Update map
                        if self.frame_count % 5 == 0:  # Update map every 5 frames
                            self.mapper.update_from_depth(
                                self.current_depth,
                                self.camera_matrix,
                                self.current_pose
                            )
                        
                        # Publish odometry
                        self.publish_odometry()
                        self.publish_trajectory()
                        
                        # Log progress
                        if self.frame_count % 30 == 0:
                            self.get_logger().info(
                                f'Frame {self.frame_count}: Position=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}), '
                                f'Matches={len(matches)}'
                            )
        
        # Store current frame as previous
        self.prev_rgb = self.current_rgb.copy()
        self.prev_depth = self.current_depth.copy()
        self.feature_tracker.prev_keypoints = keypoints
        self.feature_tracker.prev_descriptors = descriptors
    
    def publish_odometry(self):
        """Publish odometry message"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'camera_link'
        
        # Position
        msg.pose.pose.position.x = self.current_pose[0, 3]
        msg.pose.pose.position.y = self.current_pose[1, 3]
        msg.pose.pose.position.z = self.current_pose[2, 3]
        
        # Orientation
        rotation = R.from_matrix(self.current_pose[:3, :3])
        quat = rotation.as_quat()  # [x, y, z, w]
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        
        self.odom_pub.publish(msg)
        
        # Publish TF
        if self.get_parameter('publish_tf').value:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = msg.child_frame_id
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)
    
    def publish_trajectory(self):
        """Publish trajectory path"""
        if len(self.trajectory) < 2:
            return
        
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        for pos in self.trajectory:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        self.path_pub.publish(msg)
    
    def publish_map(self):
        """Publish occupancy grid map"""
        msg = self.mapper.get_occupancy_grid_msg(
            frame_id='map',
            stamp=self.get_clock().now().to_msg()
        )
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CustomSLAM()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()