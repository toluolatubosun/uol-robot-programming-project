"""
Object Detection Node

ROS2 node for YOLOv8-based object detection with 3D positioning in global frame.

External Resources:
- ROS2 (rclpy): https://docs.ros.org/en/humble/
- Ultralytics YOLOv8: https://github.com/ultralytics/ultralytics
  - Pre-trained model for object detection
  - CUDA/GPU acceleration support
- OpenCV (cv2): https://opencv.org/
  - Image visualization and processing
- PyTorch: https://pytorch.org/
  - Deep learning framework for YOLO inference
- cv_bridge: http://wiki.ros.org/cv_bridge
  - ROS-OpenCV image conversion
- image_geometry (PinholeCameraModel): http://wiki.ros.org/image_geometry
  - Camera intrinsics and 3D projection
- TF2 (tf2_ros, tf2_geometry_msgs): https://docs.ros.org/en/humble/Concepts/About-Tf2.html
  - Coordinate frame transformations (camera frame → odom frame)
- NumPy: https://numpy.org/
  - Numerical operations for 3D geometry

Algorithms:
- Bounding box center projection to 3D using depth data
- Pinhole camera model for pixel-to-ray projection
- TF2 coordinate transformation pipeline

Author: John Olatubosun
Date: January 2026
"""

import os
import cv2
import math
import rclpy
import torch
import numpy as np
from rclpy import qos
from rclpy.node import Node
from ultralytics import YOLO
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

# ROS libraries
import image_geometry
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

# ROS Messages
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header
from std_srvs.srv import SetBool


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        
        self.bridge = CvBridge()
        
        # Detection control
        self.detection_enabled = True
        
        # Get the path from the installed package share directory
        package_share_dir = get_package_share_directory('john_bot')
        model_path = os.path.join(package_share_dir, 'yolo_model_weights', 'combined_dataset_best.pt')
        self.model = YOLO(model_path)
        
        # Use GPU if available, fallback to CPU
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        if torch.cuda.is_available():
            cudnn_status = "enabled" if torch.backends.cudnn.enabled else "disabled"
            self.get_logger().info(f'Loaded YOLO model from {model_path} (GPU: {torch.cuda.get_device_name(0)}, cuDNN: {cudnn_status})')
        else:
            self.get_logger().info(f'Loaded YOLO model from {model_path} (using CPU)')
        
        # Camera models and data
        self.ccamera_model = None
        self.dcamera_model = None
        self.latest_rgb = None
        self.latest_depth = None
        self.color2depth_aspect = None
        
        # Frames
        self.global_frame = 'odom'
        self.camera_frame = 'depth_link'
        
        # TF functionality
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        camera_qos = qos.QoSProfile(
            reliability=qos.ReliabilityPolicy.RELIABLE,
            history=qos.HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribers - Camera Info
        self.ccamera_info_sub = self.create_subscription(
            CameraInfo,
            '/limo/depth_camera_link/camera_info',
            self.ccamera_info_callback,
            qos_profile=qos.qos_profile_sensor_data
        )
        
        self.dcamera_info_sub = self.create_subscription(
            CameraInfo,
            '/limo/depth_camera_link/depth/camera_info',
            self.dcamera_info_callback,
            qos_profile=qos.qos_profile_sensor_data
        )
        
        # Subscribers - Images
        self.camera_sub = self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.camera_callback,
            qos_profile=camera_qos
        )
        
        self.depth_camera_sub = self.create_subscription(
            Image,
            '/limo/depth_camera_link/depth/image_raw',
            self.depth_camera_callback,
            qos_profile=camera_qos
        )
        
        # Publishers
        self.detections_pub = self.create_publisher(
            Detection3DArray,
            '/detected_objects',
            10
        )
        
        self.poses_pub = self.create_publisher(
            PoseArray,
            '/detected_objects_poses',
            10
        )
        
        # Service to enable/disable detection
        self.enable_service = self.create_service(
            SetBool,
            'enable_detection',
            self.enable_detection_callback
        )
        
        self.get_logger().info('Detection Node initialized')
    
    def enable_detection_callback(self, request, response):
        """Service callback to enable/disable detection"""
        self.detection_enabled = request.data
        status = "enabled" if self.detection_enabled else "disabled"
        self.get_logger().info(f'Object detection {status}')
        response.success = True
        response.message = f'Detection {status}'
        return response

    def color2depth_calc(self):
        """Calculate aspect ratio between color and depth cameras"""
        if self.color2depth_aspect is None and self.ccamera_model and self.dcamera_model:
            self.color2depth_aspect = (math.atan2(self.ccamera_model.width, 2 * self.ccamera_model.fx()) / self.ccamera_model.width) \
                / (math.atan2(self.dcamera_model.width, 2 * self.dcamera_model.fx()) / self.dcamera_model.width)

    def image2camera_tf(self, image_coords, image_color, image_depth):
        """Transform from image coordinates to camera frame 3D coordinates"""
        # Transform from color to depth coordinates
        depth_coords = np.array(image_depth.shape[:2])/2 + (np.array(image_coords) - np.array(image_color.shape[:2])/2)*self.color2depth_aspect
        
        # Boundary checking
        depth_coords[0] = np.clip(depth_coords[0], 0, image_depth.shape[0]-1)
        depth_coords[1] = np.clip(depth_coords[1], 0, image_depth.shape[1]-1)
        
        # Get the depth reading at the centroid location
        depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])]
        
        # Check for valid depth
        if depth_value <= 0 or np.isnan(depth_value) or np.isinf(depth_value):
            return None
        
        # Calculate object's 3D location in camera coords
        camera_coords = np.array(self.ccamera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])))
        camera_coords /= camera_coords[2]  # Normalize so that z = 1
        camera_coords = camera_coords * depth_value  # Multiply by depth
        
        pose = Pose(
            position=Point(x=camera_coords[0], y=camera_coords[1], z=camera_coords[2]),
            orientation=Quaternion(w=1.0)
        )
        return pose
    
    def ccamera_info_callback(self, data):
        """Callback for color camera info"""
        if self.ccamera_model is None:
            self.ccamera_model = image_geometry.PinholeCameraModel()
            self.ccamera_model.fromCameraInfo(data)
            self.color2depth_calc()
            self.get_logger().info('Color camera model initialized')

    def dcamera_info_callback(self, data):
        """Callback for depth camera info"""
        if self.dcamera_model is None:
            self.dcamera_model = image_geometry.PinholeCameraModel()
            self.dcamera_model.fromCameraInfo(data)
            self.color2depth_calc()
            self.get_logger().info('Depth camera model initialized')

    def depth_camera_callback(self, msg):
        """Callback for Depth images"""
        self.get_logger().info('Depth callback triggered!', throttle_duration_sec=10.0)
        
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            depth_display = self.latest_depth * 1.0/10.0
            # cv2.imshow("Depth Camera", depth_display)
            # cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Depth callback error: {str(e)}')

    def camera_callback(self, msg):
        """Callback for RGB images"""
        self.get_logger().info('RGB callback triggered!', throttle_duration_sec=10.0)
        
        # Check if detection is enabled
        if not self.detection_enabled:
            return
        
        # Wait for camera models and depth image
        if self.color2depth_aspect is None or self.latest_depth is None:
            return

        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model(self.latest_rgb, verbose=False, conf=0.45)
            
            # Draw bounding boxes on image
            annotated_frame = results[0].plot()
            
            # Process detections and publish
            self.publish_detections(results[0], msg.header)
            
            cv2.imshow("RGB Camera - Detections", annotated_frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'RGB callback error: {str(e)}')

    def publish_detections(self, result, header):
        """Publish detected objects as Detection3DArray and PoseArray"""
        detection_array = Detection3DArray()
        detection_array.header = Header(frame_id=self.global_frame)
        
        pose_array = PoseArray()
        pose_array.header = Header(frame_id=self.global_frame)
        
        boxes = result.boxes
        
        for box in boxes:
            # Get bounding box center
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            center_u = int((x1 + x2) / 2)
            center_v = int((y1 + y2) / 2)
            
            # Image coordinates (row, col)
            image_coords = (center_v, center_u)
            
            # Transform to camera frame
            camera_pose = self.image2camera_tf(image_coords, self.latest_rgb, self.latest_depth)
            
            if camera_pose is not None:
                try:
                    # Transform from camera frame to global frame (odom)
                    transform = self.tf_buffer.lookup_transform(
                        self.global_frame,
                        self.camera_frame,
                        rclpy.time.Time()
                    )
                    global_pose = do_transform_pose(camera_pose, transform)
                    
                    # Create Detection3D message
                    detection = Detection3D()
                    detection.header = Header(frame_id=self.global_frame)
                    
                    class_id = int(box.cls[0])
                    class_name = result.names[class_id]
                    
                    # Object hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = f"{class_id}:{class_name}"
                    hypothesis.hypothesis.score = float(box.conf[0])
                    
                    # Position (in global frame)
                    hypothesis.pose.pose = global_pose
                    
                    detection.results.append(hypothesis)
                    detection_array.detections.append(detection)
                    
                    # Create Pose for PoseArray
                    pose_array.poses.append(global_pose)
                    
                    # Log detection
                    class_name = result.names[int(box.cls[0])]
                    self.get_logger().info(
                        f'Detected {class_name} at global ({global_pose.position.x:.2f}, '
                        f'{global_pose.position.y:.2f}, {global_pose.position.z:.2f}) '
                        f'with confidence {float(box.conf[0]):.2f}',
                        throttle_duration_sec=2.0
                    )
                    
                except Exception as e:
                    self.get_logger().warn(f'TF transform failed: {str(e)}', throttle_duration_sec=5.0)
        
        # Publish if we have detections
        if len(detection_array.detections) > 0:
            self.detections_pub.publish(detection_array)
            self.poses_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    
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