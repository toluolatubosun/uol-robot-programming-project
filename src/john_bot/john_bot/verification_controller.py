"""
Verification Controller Node

ROS2 node for multi-angle object verification with autonomous navigation.

External Resources:
- ROS2 (rclpy): https://docs.ros.org/en/humble/
- Nav2 Navigation Stack: https://navigation.ros.org/
  - NavigateToPose action for autonomous navigation
  - Costmap2D for collision checking
- ROS2 QoS (TRANSIENT_LOCAL durability for late-joining nodes)
- Python threading for async verification workflow

Algorithms:
- Greedy nearest neighbor for route optimization
- Multi-angle verification strategy (0°, 120°, 240° coverage)
- Costmap-based collision checking with robot footprint

Author: John Olatubsoun
Date: January 2026
"""

import os
import json
import math
import time
import rclpy
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from vision_msgs.msg import Detection3DArray
from nav_msgs.msg import OccupancyGrid, Odometry


class VerificationController(Node):
    def __init__(self):
        super().__init__('verification_controller')
        
        # Parameters
        self.declare_parameter('standoff_distance', 1.5)
        self.declare_parameter('object_verification_duration', 5.0)
        self.declare_parameter('object_match_distance_tolerance', 1.0)
        
        self.standoff_distance = self.get_parameter('standoff_distance').value
        self.object_verification_duration = self.get_parameter('object_verification_duration').value
        self.object_match_distance_tolerance = self.get_parameter('object_match_distance_tolerance').value
        
        # State
        self.world_id = None
        self.trial_id = None
        self.verification_active = False
        self.verification_detections = []
        self.costmap = None
        self.initial_pose = None
        self.verification_start_time = None
        self.verification_end_time = None 
        
        # Subscribe to exploration status
        status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_sub = self.create_subscription(
            String, '/explore/status', self.status_callback, status_qos)
        
        # Subscribe to trial config (TRANSIENT_LOCAL to receive late messages)
        config_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.config_sub = self.create_subscription(
            String, '/trial_config', self.config_callback, config_qos)
        
        # Subscribe to detections during verification
        self.detections_sub = self.create_subscription(
            Detection3DArray, '/detected_objects', self.detections_callback, 10)
        
        # Subscribe to costmap
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        
        # Subscribe to odom to capture initial position
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Service client to control detection
        self.detection_control_client = self.create_client(SetBool, 'enable_detection')
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.get_logger().info('Verification Controller initialized')
        self.get_logger().info(f'Standoff: {self.standoff_distance}m, Timeout: {self.object_verification_duration}s')
        self.get_logger().info('Waiting for exploration to complete...')
    
    def odom_callback(self, msg):
        """Capture initial pose once"""
        if self.initial_pose is None:
            self.initial_pose = msg.pose.pose
            self.get_logger().info(
                f'Captured initial pose: '
                f'({self.initial_pose.position.x:.2f}, {self.initial_pose.position.y:.2f})'
            )
            # Unsubscribe after capturing initial pose
            self.destroy_subscription(self.odom_sub)
    
    def config_callback(self, msg):
        """Receive world and trial ID"""
        if self.world_id is None:
            parts = msg.data.split(',')
            self.world_id = parts[0]
            self.trial_id = parts[1]
            self.get_logger().info(f'Config: World={self.world_id}, Trial={self.trial_id}')
    
    def status_callback(self, msg):
        """React to exploration status"""
        self.get_logger().info(f'Status: {msg.data}')
        
        if msg.data == "returned_to_origin" and not self.verification_active:
            self.get_logger().info('STARTING VERIFICATION PHASE')
            threading.Thread(target=self.start_verification, daemon=True).start()
    
    def costmap_callback(self, msg):
        """Store costmap for collision checking"""
        self.costmap = msg
    
    def detections_callback(self, msg):
        """Store detections during verification"""
        if self.verification_active:
            self.verification_detections.extend(msg.detections)
    
    def optimize_verification_route(self, detections):
        """Sort detections by proximity using greedy nearest neighbor"""
        if len(detections) <= 1:
            return detections
        
        # Start from robot's current position (initial pose captured from odom)
        if self.initial_pose is not None:
            current_pos = {
                'x': self.initial_pose.position.x,
                'y': self.initial_pose.position.y,
                'z': self.initial_pose.position.z
            }
        else:
            # Fallback to origin if initial pose not captured
            self.get_logger().warn('Initial pose not available, assuming origin for route optimization')
            current_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        ordered = []
        remaining = detections.copy()
        
        while remaining:
            # Find nearest unvisited object
            nearest_idx = 0
            min_dist = float('inf')
            
            for i, detection in enumerate(remaining):
                pos = detection['position']
                dist = math.sqrt(
                    (pos['x'] - current_pos['x'])**2 +
                    (pos['y'] - current_pos['y'])**2
                )
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = i
            
            # Visit nearest
            nearest = remaining.pop(nearest_idx)
            ordered.append(nearest)
            current_pos = nearest['position']

        return ordered

    def start_verification(self):
        """Main verification workflow"""
        if self.world_id is None or self.trial_id is None:
            self.get_logger().error('No trial config received yet!')
            return
    
        self.verification_active = True
        self.verification_start_time = self.get_clock().now() 
        
        # Load detection results
        results_file = f'results/exploration/World_{self.world_id}/trial_{self.trial_id}.json'
        
        if not os.path.exists(results_file):
            self.get_logger().error(f'Results file not found: {results_file}')
            return
        
        with open(results_file, 'r') as f:
            results = json.load(f)
        
        detections = results['detections']
        self.get_logger().info(f'Loaded {len(detections)} detections to verify')

        # OPTIMIZE ROUTE - visit closest objects first
        detections = self.optimize_verification_route(detections)
        self.get_logger().info('Optimized verification route for efficiency')
        
        # Disable continuous detection
        self.set_detection_enabled(False)
        
        # Verify each detection
        verified_detections = []
        for i, detection in enumerate(detections):
            self.get_logger().info(f'Verifying {i+1}/{len(detections)}...')
            
            if self.verify_object(detection):
                verified_detections.append(detection)
                self.get_logger().info('✓ Verified')
            else:
                self.get_logger().warn('✗ Failed - removing false positive')
        
        # Re-enable continuous detection
        self.set_detection_enabled(True)
        
        self.verification_end_time = self.get_clock().now()

        # Save updated results
        results['detections'] = verified_detections
        results['total_detections'] = len(verified_detections)
        results['verification_complete'] = True
        results['verification_start_time'] = self.verification_start_time.to_msg().sec
        results['verification_end_time'] = self.verification_end_time.to_msg().sec
        results['verification_duration_seconds'] = (self.verification_end_time - self.verification_start_time).nanoseconds / 1e9
        
        with open(results_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.get_logger().info('VERIFICATION COMPLETE')
        self.get_logger().info(f'Confirmed: {len(verified_detections)}/{len(detections)}')
        
        # Return to origin using captured initial pose
        if self.initial_pose is not None:
            self.get_logger().info('Returning to origin...')
            origin_pose = PoseStamped()
            origin_pose.header.frame_id = 'odom'
            origin_pose.header.stamp = rclpy.time.Time().to_msg()
            origin_pose.pose = self.initial_pose
            
            if self.navigate_to_pose(origin_pose):
                self.get_logger().info('Successfully returned to origin')
            else:
                self.get_logger().warn('Failed to return to origin')
        else:
            self.get_logger().warn('Initial pose not captured, cannot return to origin')
        
        self.verification_active = False
    
    def verify_object(self, detection):
        """Verify a single object by trying multiple viewing angles"""
        object_pos = detection['position']
        
        # Try 3 angles for 360° coverage: 0°, 120°, 240°
        angles_to_try = [0, 120, 240]
        
        for angle_deg in angles_to_try:
            self.get_logger().info(f'  Trying angle {angle_deg}°...')
            
            # DISABLE DETECTION BEFORE NAVIGATION
            self.set_detection_enabled(False)
            
            # Calculate safe standoff position for this angle
            standoff_pose = self.calculate_safe_standoff(object_pos, angle_deg)
            
            if standoff_pose is None:
                self.get_logger().warn(f'  No valid standoff at {angle_deg}°, trying next angle')
                continue
            
            # Navigate to standoff
            if not self.navigate_to_pose(standoff_pose):
                self.get_logger().warn(f'  Navigation failed at {angle_deg}°, trying next angle')
                continue
            
            # Clear previous detections
            self.verification_detections = []
            
            # ENABLE detection for verification
            self.set_detection_enabled(True)
            
            # Wait for detections while processing callbacks
            end_time = time.time() + self.object_verification_duration
            while time.time() < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # DISABLE detection
            self.set_detection_enabled(False)
            
            # Check if detected
            if self.match_detection(object_pos, detection['class_id']):
                self.get_logger().info(f'  ✓ Detected from {angle_deg}°!')
                return True
            else:
                self.get_logger().warn(f'  ✗ Not detected from {angle_deg}°')
        
        # All angles failed
        return False

    def calculate_safe_standoff(self, object_pos, primary_angle):
        """Calculate standoff for a specific angle, with fallback angles if blocked"""
        # Try primary angle first, then nearby angles if blocked
        fallback_offsets = [0, 45, -45, 90, -90]  # Try within ±90° of primary
        
        for offset in fallback_offsets:
            angle_deg = (primary_angle + offset) % 360
            angle_rad = math.radians(angle_deg)
            
            x = object_pos['x'] + self.standoff_distance * math.cos(angle_rad)
            y = object_pos['y'] + self.standoff_distance * math.sin(angle_rad)
            
            if self.is_position_valid(x, y):
                pose = PoseStamped()
                pose.header.frame_id = 'odom'
                pose.header.stamp = rclpy.time.Time().to_msg()
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                
                # Face the object
                yaw = math.atan2(object_pos['y'] - y, object_pos['x'] - x)
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
                
                if offset != 0:
                    self.get_logger().info(f'    Using fallback angle {angle_deg}° (primary {primary_angle}° blocked)')
                
                return pose
        
        return None
    
    def is_position_valid(self, x, y):
        """Check if position is free space with robot footprint clearance"""
        if self.costmap is None:
            return True
        
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y
        resolution = self.costmap.info.resolution
        
        robot_radius = 0.25  # meters (0.1m robot + 0.15m safety)
        cells_to_check = int(robot_radius / resolution)
        
        grid_x = int((x - origin_x) / resolution)
        grid_y = int((y - origin_y) / resolution)
        
        # Check circular area around position
        for dx in range(-cells_to_check, cells_to_check + 1):
            for dy in range(-cells_to_check, cells_to_check + 1):
                # Only check cells within robot's circular footprint
                if math.sqrt(dx*dx + dy*dy) > cells_to_check:
                    continue
                    
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                # Check bounds
                if not (0 <= check_x < self.costmap.info.width and 
                        0 <= check_y < self.costmap.info.height):
                    return False
                
                # Check cost
                index = check_y * self.costmap.info.width + check_x
                cost = self.costmap.data[index]
                
                if cost >= 50:  # Occupied or unknown
                    return False
        
        return True
    
    def navigate_to_pose(self, pose_stamped):
        """Navigate using Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose_stamped
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        timeout = rclpy.duration.Duration(seconds=10.0)
        start_time = self.get_clock().now()
        
        while not send_goal_future.done():
            if (self.get_clock().now() - start_time) > timeout:
                self.get_logger().error('Goal send timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return False
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        result = result_future.result()
        return result.status == 4  # SUCCEEDED
    
    def match_detection(self, expected_pos, expected_class):
        """Check if object was detected"""
        for detection in self.verification_detections:
            if len(detection.results) == 0:
                continue
            
            hypothesis = detection.results[0]
            detected_pos = hypothesis.pose.pose.position
            detected_class = hypothesis.hypothesis.class_id
            
            # Match class
            if detected_class != expected_class:
                continue
            
            # Match position
            distance = math.sqrt(
                (detected_pos.x - expected_pos['x'])**2 +
                (detected_pos.y - expected_pos['y'])**2 +
                (detected_pos.z - expected_pos['z'])**2
            )
            
            if distance < self.object_match_distance_tolerance:
                return True
        
        return False
    
    def set_detection_enabled(self, enabled):
        """Control object detection"""
        if not self.detection_control_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Detection service unavailable')
            return False
        
        request = SetBool.Request()
        request.data = enabled
        
        future = self.detection_control_client.call_async(request)
        
        # Wait for response without blocking the node
        start_time = self.get_clock().now()
        timeout = rclpy.duration.Duration(seconds=5.0)
        
        while not future.done():
            if (self.get_clock().now() - start_time) > timeout:
                self.get_logger().error('Service call timeout')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        try:
            response = future.result()
            status = "enabled" if enabled else "disabled"
            self.get_logger().info(f'Detection {status}: {response.message}')
            return response.success
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = VerificationController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()