#!/usr/bin/env python3
"""
SLAM Debug Visualizer
Shows feature matches, depth overlay, and tracking quality in real-time
Place in: custom_slam/custom_slam/visualizer_node.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np

class SLAMVisualizer(Node):
    """Visualizes SLAM debugging information"""
    
    def __init__(self):
        super().__init__('slam_visualizer')
        
        self.bridge = CvBridge()
        
        # State
        self.rgb_image = None
        self.depth_image = None
        self.odometry = None
        self.prev_rgb = None
        
        # Feature detector
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)
        self.prev_kp = None
        self.prev_desc = None
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/slam/odometry',
            self.odom_callback, 10
        )
        
        # Timer for visualization
        self.create_timer(0.033, self.visualize)  # ~30 Hz
        
        self.get_logger().info('SLAM Visualizer started')
    
    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    def depth_callback(self, msg):
        depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_image = depth_mm.astype(np.float32) / 1000.0
    
    def odom_callback(self, msg):
        self.odometry = msg
    
    def visualize(self):
        if self.rgb_image is None:
            return
        
        # Create visualization panels
        vis_image = self.rgb_image.copy()
        
        # Panel 1: Feature matches
        if self.prev_rgb is not None:
            gray = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)
            kp, desc = self.orb.detectAndCompute(gray, None)
            
            if self.prev_desc is not None and desc is not None:
                matches = self.matcher.knnMatch(self.prev_desc, desc, k=2)
                
                # Ratio test
                good_matches = []
                for match_pair in matches:
                    if len(match_pair) == 2:
                        m, n = match_pair
                        if m.distance < 0.75 * n.distance:
                            good_matches.append(m)
                
                # Draw matches
                match_img = cv2.drawMatches(
                    self.prev_rgb, self.prev_kp,
                    self.rgb_image, kp,
                    good_matches[:50],  # Show top 50 matches
                    None,
                    flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS
                )
                
                # Resize to fit
                h, w = match_img.shape[:2]
                scale = 800 / w
                match_img = cv2.resize(match_img, (800, int(h * scale)))
                
                cv2.imshow('Feature Matches', match_img)
                
                # Update status text
                cv2.putText(vis_image, f'Matches: {len(good_matches)}', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            self.prev_kp = kp
            self.prev_desc = desc
        
        # Panel 2: Depth overlay
        if self.depth_image is not None:
            # Normalize depth for visualization
            depth_vis = self.depth_image.copy()
            depth_vis = np.clip(depth_vis, 0, 8.0)  # Clip to 8 meters
            depth_vis = (depth_vis / 8.0 * 255).astype(np.uint8)
            depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            
            # Blend with RGB
            overlay = cv2.addWeighted(self.rgb_image, 0.6, depth_colored, 0.4, 0)
            cv2.imshow('Depth Overlay', overlay)
        
        # Panel 3: Info overlay
        if self.odometry is not None:
            pos = self.odometry.pose.pose.position
            cv2.putText(vis_image, f'X: {pos.x:.2f}m', 
                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(vis_image, f'Y: {pos.y:.2f}m', 
                       (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(vis_image, f'Z: {pos.z:.2f}m', 
                       (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Draw feature points on main image
        gray = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)
        kp, _ = self.orb.detectAndCompute(gray, None)
        vis_image = cv2.drawKeypoints(vis_image, kp, None, 
                                      color=(0, 255, 0), 
                                      flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        cv2.imshow('SLAM View', vis_image)
        cv2.waitKey(1)
        
        self.prev_rgb = self.rgb_image.copy()


def main(args=None):
    rclpy.init(args=args)
    node = SLAMVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()