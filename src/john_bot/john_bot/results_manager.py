"""
Results Manager Node

ROS2 node for detection aggregation, spatial clustering, and JSON output generation.

External Resources:
- ROS2 (rclpy): https://docs.ros.org/en/humble/
- ROS2 QoS (TRANSIENT_LOCAL durability for persistent messages): 
  https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- vision_msgs (Detection3DArray): http://wiki.ros.org/vision_msgs
- geometry_msgs (PoseArray, Pose): http://wiki.ros.org/geometry_msgs
- Python json module for data serialization

Algorithms:
- Spatial clustering using 3D Euclidean distance for de-duplication
- Duplicate detection: updates existing entry with higher-confidence observation

Author: John Olatubosun
Date: January 2026
"""

import os
import json
import math
import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Header, String
from explore_lite.msg import ExploreStatus
from vision_msgs.msg import Detection3DArray


class ResultsManager(Node):
    def __init__(self):
        super().__init__('results_manager')
        
        # Declare parameters
        self.declare_parameter('world_id', 'A')
        self.declare_parameter('trial_id', 1)
        self.declare_parameter('clustering_threshold', 2.0)
        
        # Get parameters
        self.world_id = self.get_parameter('world_id').value
        self.trial_id = self.get_parameter('trial_id').value
        self.clustering_threshold = self.get_parameter('clustering_threshold').value
        
        # Storage
        self.detected_objects = []  # List of unique detections
        self.start_time = self.get_clock().now()
        self.final_save_done = False
        
        # Results directory setup
        self.results_dir = f'results/exploration/World_{self.world_id}'
        os.makedirs(self.results_dir, exist_ok=True)
        
        self.output_file = os.path.join(
            self.results_dir,
            f'trial_{self.trial_id}.json'
        )
        
        # Subscribers
        self.detections_sub = self.create_subscription(
            Detection3DArray,
            '/detected_objects',
            self.detections_callback,
            qos_profile=qos.qos_profile_sensor_data
        )
        
        # Subscribe to exploration status
        status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_sub = self.create_subscription(
            ExploreStatus,
            '/explore/status',
            self.status_callback,
            status_qos
        )
        
        # Publisher for de-duplicated poses (for RViz visualization)
        self.deduplicated_poses_pub = self.create_publisher(
            PoseArray,
            '/deduplicated_objects_poses',
            10
        )

        # Publisher for trial configuration (TRANSIENT_LOCAL so late subscribers get it)
        config_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.config_pub = self.create_publisher(String, '/trial_config', config_qos)
        
        # Timer to periodically save results (every 10 seconds)
        self.save_timer = self.create_timer(10.0, self.save_results)
        
        # Timer to publish config (every 1 second for reliability)
        self.config_timer = self.create_timer(1.0, self.publish_config)
        
        self.get_logger().info(
            f'Results Manager initialized: World={self.world_id}, '
            f'Trial={self.trial_id}, Output={self.output_file}'
        )
    
    def status_callback(self, msg):
        """React to exploration status"""
        self.get_logger().info(f'Exploration status: {msg.status}')
        
        if msg.status == "returned_to_origin" and not self.final_save_done:
            self.get_logger().info('Exploration complete - doing final save and stopping periodic saves')
            self.save_results()
            self.final_save_done = True
            # Stop periodic saves - verification will modify the file directly
            self.save_timer.cancel()
    
    def publish_config(self):
        """Publish trial configuration for other nodes"""
        msg = String()
        msg.data = f"{self.world_id},{self.trial_id}"
        self.config_pub.publish(msg)
    
    def calculate_distance(self, pos1, pos2):
        """Calculate 3D Euclidean distance between two positions"""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def detections_callback(self, msg):
        """Process incoming detections and perform clustering"""
        for detection in msg.detections:
            if len(detection.results) == 0:
                continue
            
            # Get the first (best) hypothesis
            hypothesis = detection.results[0]
            new_pose = hypothesis.pose.pose
            class_id = hypothesis.hypothesis.class_id
            confidence = hypothesis.hypothesis.score
            
            # Check if this detection is a duplicate (clustering)
            is_duplicate = False
            for existing_obj in self.detected_objects:
                existing_pos = existing_obj['position']
                
                # Calculate distance
                class DummyPoint:
                    def __init__(self, x, y, z):
                        self.x = x
                        self.y = y
                        self.z = z
                
                existing_point = DummyPoint(
                    existing_pos['x'],
                    existing_pos['y'],
                    existing_pos['z']
                )
                
                distance = self.calculate_distance(new_pose.position, existing_point)
                
                if distance < self.clustering_threshold:
                    is_duplicate = True
                    
                    # Update with higher confidence (and update class if needed)
                    if confidence > existing_obj['confidence']:
                        existing_obj['class_id'] = class_id
                        existing_obj['confidence'] = confidence
                        existing_obj['position'] = {
                            'x': new_pose.position.x,
                            'y': new_pose.position.y,
                            'z': new_pose.position.z
                        }
                        self.get_logger().info(
                            f'Updated detection: Class={class_id}, '
                            f'New confidence={confidence:.2f}',
                            throttle_duration_sec=2.0
                        )
                    break
            
            # Add new unique detection
            if not is_duplicate:
                detection_obj = {
                    'class_id': class_id,
                    'position': {
                        'x': new_pose.position.x,
                        'y': new_pose.position.y,
                        'z': new_pose.position.z
                    },
                    'confidence': confidence,
                    'timestamp': self.get_clock().now().to_msg().sec
                }
                
                self.detected_objects.append(detection_obj)
                
                self.get_logger().info(
                    f'New detection: Class={class_id}, '
                    f'Position=({new_pose.position.x:.2f}, {new_pose.position.y:.2f}, {new_pose.position.z:.2f}), '
                    f'Confidence={confidence:.2f}, Total count={len(self.detected_objects)}'
                )
        
        # Publish de-duplicated poses for visualization
        self.publish_deduplicated_poses(msg.header.frame_id)
    
    def publish_deduplicated_poses(self, frame_id):
        """Publish PoseArray of all unique detections for RViz"""
        pose_array = PoseArray()
        pose_array.header = Header(frame_id=frame_id, stamp=self.get_clock().now().to_msg())
        
        for obj in self.detected_objects:
            from geometry_msgs.msg import Pose, Point, Quaternion
            pose = Pose()
            pose.position = Point(
                x=obj['position']['x'],
                y=obj['position']['y'],
                z=obj['position']['z']
            )
            pose.orientation = Quaternion(w=1.0)
            pose_array.poses.append(pose)
        
        self.deduplicated_poses_pub.publish(pose_array)
    
    def save_results(self):
        """Save current results to JSON file"""
        if len(self.detected_objects) == 0:
            return
        
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        results = {
            'world_id': self.world_id,
            'trial_id': self.trial_id,
            'exploration_start_time': self.start_time.to_msg().sec,
            'exploration_end_time': self.get_clock().now().to_msg().sec,
            'exploration_duration_seconds': elapsed_time,
            'clustering_threshold': self.clustering_threshold,
            'total_detections': len(self.detected_objects),
            'detections': self.detected_objects
        }
        
        try:
            with open(self.output_file, 'w') as f:
                json.dump(results, f, indent=2)
            
            self.get_logger().info(
                f'Saved {len(self.detected_objects)} detections to {self.output_file}',
                throttle_duration_sec=10.0
            )
        except Exception as e:
            self.get_logger().error(f'Failed to save results: {str(e)}')
    
    def shutdown_hook(self):
        """Save final results on shutdown (only if not already done)"""
        if not self.final_save_done:
            self.get_logger().info('Shutdown detected - saving final results...')
            self.save_results()
            self.final_save_done = True
        
        # Print summary
        self.get_logger().info(f'Final Summary - World {self.world_id}, Trial {self.trial_id}')
        self.get_logger().info(f'Total unique detections: {len(self.detected_objects)}')
        
        # Count by class
        class_counts = {}
        for obj in self.detected_objects:
            class_id = obj['class_id']
            class_counts[class_id] = class_counts.get(class_id, 0) + 1
        
        for class_id, count in class_counts.items():
            self.get_logger().info(f'  Class {class_id}: {count}')


def main(args=None):
    rclpy.init(args=args)
    node = ResultsManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()