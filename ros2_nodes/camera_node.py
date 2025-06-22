#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped

import cv2
import numpy as np
from cv_bridge import CvBridge
import pyrealsense2 as rs

import os
import json
from datetime import datetime


class CameraNode(Node):
    """
    ROS 2 node for Intel RealSense camera integration.
    Handles image capture, processing, and defect detection simulation.
    """
    
    def __init__(self):
        super().__init__('camera_node')
        
        # Initialize callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers
        self.color_image_pub = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            10
        )
        
        self.depth_image_pub = self.create_publisher(
            Image,
            '/camera/depth/image_raw',
            10
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/color/camera_info',
            10
        )
        
        self.inspection_result_pub = self.create_publisher(
            String,
            '/inspection_result',
            10
        )
        
        # Subscribers
        self.capture_request_sub = self.create_subscription(
            String,
            '/capture_image',
            self.capture_image_callback,
            10
        )
        
        self.inspection_request_sub = self.create_subscription(
            String,
            '/run_inspection',
            self.run_inspection_callback,
            10
        )
        
        # RealSense pipeline
        self.pipeline = None
        self.config = None
        self.device = None
        
        # Camera parameters
        self.color_width = 640
        self.color_height = 480
        self.depth_width = 640
        self.depth_height = 480
        self.fps = 30
        
        # Inspection parameters
        self.defect_threshold = 0.1
        self.inspection_areas = [
            {'name': 'area_1', 'x': 100, 'y': 100, 'w': 200, 'h': 200},
            {'name': 'area_2', 'x': 350, 'y': 150, 'w': 150, 'h': 150},
            {'name': 'area_3', 'x': 200, 'y': 300, 'w': 180, 'h': 120}
        ]
        
        # Initialize camera
        self._initialize_camera()
        
        # Timer for publishing camera info
        self.timer = self.create_timer(1.0, self.publish_camera_info)
        
        self.get_logger().info('Camera Node initialized')
        
    def _initialize_camera(self):
        """Initialize RealSense camera or use simulation mode."""
        try:
            # Try to initialize RealSense camera
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # Configure streams
            self.config.enable_stream(
                rs.stream.color, 
                self.color_width, 
                self.color_height, 
                rs.format.bgr8, 
                self.fps
            )
            self.config.enable_stream(
                rs.stream.depth, 
                self.depth_width, 
                self.depth_height, 
                rs.format.z16, 
                self.fps
            )
            
            # Start pipeline
            profile = self.pipeline.start(self.config)
            self.device = profile.get_device()
            
            # Get camera intrinsics
            color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
            self.color_intrinsics = color_profile.get_intrinsics()
            
            self.get_logger().info('RealSense camera initialized successfully')
            self.simulation_mode = False
            
        except Exception as e:
            self.get_logger().warn(f'RealSense camera not available: {str(e)}')
            self.get_logger().info('Running in simulation mode')
            self.simulation_mode = True
            self._setup_simulation()
    
    def _setup_simulation(self):
        """Setup simulation mode with synthetic images."""
        # Create synthetic camera intrinsics
        self.color_intrinsics = type('Intrinsics', (), {
            'width': self.color_width,
            'height': self.color_height,
            'fx': 525.0,
            'fy': 525.0,
            'ppx': self.color_width / 2,
            'ppy': self.color_height / 2
        })()
        
        # Create a synthetic image for simulation
        self.synthetic_image = np.ones((self.color_height, self.color_width, 3), dtype=np.uint8) * 128
        self.synthetic_depth = np.ones((self.depth_height, self.depth_width), dtype=np.uint16) * 1000
        
        # Add some synthetic features
        cv2.rectangle(self.synthetic_image, (100, 100), (300, 300), (255, 0, 0), 2)
        cv2.circle(self.synthetic_image, (400, 200), 50, (0, 255, 0), -1)
    
    def capture_image_callback(self, msg):
        """Callback for image capture requests."""
        try:
            self.get_logger().info('Capturing image...')
            
            if self.simulation_mode:
                # Use synthetic image
                color_image = self.synthetic_image.copy()
                depth_image = self.synthetic_depth.copy()
            else:
                # Capture from RealSense
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame or not depth_frame:
                    self.get_logger().error('Failed to capture frames')
                    return
                
                color_image = np.asanyarray(color_frame.get_data())
                depth_image = np.asanyarray(depth_frame.get_data())
            
            # Convert to ROS messages
            color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
            
            # Set timestamps
            timestamp = self.get_clock().now().to_msg()
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = "camera_color_optical_frame"
            depth_msg.header.stamp = timestamp
            depth_msg.header.frame_id = "camera_depth_optical_frame"
            
            # Publish images
            self.color_image_pub.publish(color_msg)
            self.depth_image_pub.publish(depth_msg)
            
            # Save image if requested
            if msg.data == "save":
                timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"captured_image_{timestamp_str}.jpg"
                cv2.imwrite(filename, color_image)
                self.get_logger().info(f'Image saved as {filename}')
            
            self.get_logger().info('Image captured and published')
            
        except Exception as e:
            self.get_logger().error(f'Error capturing image: {str(e)}')
    
    def run_inspection_callback(self, msg):
        """Callback for inspection requests."""
        try:
            self.get_logger().info('Running inspection...')
            
            # Capture current image
            if self.simulation_mode:
                inspection_image = self.synthetic_image.copy()
            else:
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    self.get_logger().error('Failed to capture frame for inspection')
                    return
                inspection_image = np.asanyarray(color_frame.get_data())
            
            # Run defect detection
            defects = self._detect_defects(inspection_image)
            
            # Create inspection result
            result = {
                'timestamp': datetime.now().isoformat(),
                'defects_found': len(defects),
                'defects': defects,
                'overall_result': 'PASS' if len(defects) == 0 else 'FAIL',
                'confidence': 0.95
            }
            
            # Publish result
            result_msg = String()
            result_msg.data = json.dumps(result)
            self.inspection_result_pub.publish(result_msg)
            
            # Save inspection image
            timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"inspection_{timestamp_str}.jpg"
            self._draw_inspection_results(inspection_image, defects, filename)
            
            self.get_logger().info(f'Inspection completed: {result["overall_result"]}')
            
        except Exception as e:
            self.get_logger().error(f'Error running inspection: {str(e)}')
    
    def _detect_defects(self, image):
        """Simulate defect detection using OpenCV."""
        defects = []
        
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Apply edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Analyze each inspection area
        for area in self.inspection_areas:
            x, y, w, h = area['x'], area['y'], area['w'], area['h']
            roi = gray[y:y+h, x:x+w]
            
            # Simple defect detection (simulation)
            # In real implementation, this would use ML models
            mean_intensity = np.mean(roi)
            std_intensity = np.std(roi)
            
            # Simulate defects based on intensity variations
            if std_intensity > 30 or mean_intensity < 100 or mean_intensity > 200:
                defect = {
                    'area': area['name'],
                    'type': 'intensity_anomaly',
                    'severity': 'medium',
                    'confidence': 0.85,
                    'location': {'x': x + w//2, 'y': y + h//2}
                }
                defects.append(defect)
        
        # Add some random defects for demonstration
        if np.random.random() < 0.3:  # 30% chance of defect
            defect = {
                'area': 'random_area',
                'type': 'simulated_defect',
                'severity': 'low',
                'confidence': 0.75,
                'location': {'x': np.random.randint(100, 500), 'y': np.random.randint(100, 400)}
            }
            defects.append(defect)
        
        return defects
    
    def _draw_inspection_results(self, image, defects, filename):
        """Draw inspection results on image and save."""
        result_image = image.copy()
        
        # Draw inspection areas
        for area in self.inspection_areas:
            x, y, w, h = area['x'], area['y'], area['w'], area['h']
            cv2.rectangle(result_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(result_image, area['name'], (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Draw defects
        for defect in defects:
            x, y = defect['location']['x'], defect['location']['y']
            color = (0, 0, 255) if defect['severity'] == 'high' else (0, 165, 255)
            cv2.circle(result_image, (x, y), 10, color, -1)
            cv2.putText(result_image, defect['type'], (x + 15, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Save image
        cv2.imwrite(filename, result_image)
        self.get_logger().info(f'Inspection result saved as {filename}')
    
    def publish_camera_info(self):
        """Publish camera information."""
        try:
            camera_info = CameraInfo()
            camera_info.header.stamp = self.get_clock().now().to_msg()
            camera_info.header.frame_id = "camera_color_optical_frame"
            
            # Set camera parameters
            camera_info.width = self.color_intrinsics.width
            camera_info.height = self.color_intrinsics.height
            camera_info.k = [
                self.color_intrinsics.fx, 0, self.color_intrinsics.ppx,
                0, self.color_intrinsics.fy, self.color_intrinsics.ppy,
                0, 0, 1
            ]
            camera_info.p = [
                self.color_intrinsics.fx, 0, self.color_intrinsics.ppx, 0,
                0, self.color_intrinsics.fy, self.color_intrinsics.ppy, 0,
                0, 0, 1, 0
            ]
            
            self.camera_info_pub.publish(camera_info)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing camera info: {str(e)}')
    
    def __del__(self):
        """Cleanup when node is destroyed."""
        if self.pipeline and not self.simulation_mode:
            self.pipeline.stop()


def main(args=None):
    rclpy.init(args=args)
    
    camera_node = CameraNode()
    
    # Use multi-threaded executor
    executor = MultiThreadedExecutor()
    executor.add_node(camera_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 