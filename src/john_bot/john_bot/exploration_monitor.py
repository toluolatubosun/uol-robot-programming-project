"""
Exploration Monitor Node

ROS2 node for periodic SLAM map saving during exploration with final map capture on completion.

External Resources:
- ROS2 (rclpy): https://docs.ros.org/en/humble/
- ROS2 QoS (TRANSIENT_LOCAL durability for persistent messages):
  https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html
- Nav2 Map Server (map_saver_cli): https://navigation.ros.org/configuration/packages/configuring-map-server.html
  - Subprocess execution for saving occupancy grid maps
- Python subprocess module for external process execution
- Python os module for directory management

Author: John Olatubosun
Date: January 2026
"""

import os
import rclpy
import subprocess
from rclpy.node import Node
from std_msgs.msg import String
from explore_lite.msg import ExploreStatus
from rclpy.qos import QoSProfile, DurabilityPolicy


class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__('exploration_monitor')
        
        # Parameters
        self.declare_parameter('periodic_save_interval', 30.0)  # Save every 30 seconds
        self.periodic_interval = self.get_parameter('periodic_save_interval').value
        
        # State
        self.world_id = None
        self.trial_id = None
        self.save_count = 0
        self.final_save_done = False
        
        # Subscribe to trial configuration (TRANSIENT_LOCAL to receive late messages)
        config_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.config_sub = self.create_subscription(
            String,
            '/trial_config',
            self.config_callback,
            config_qos
        )
        
        # Subscribe to exploration status
        status_qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_sub = self.create_subscription(
            ExploreStatus,
            '/explore/status',
            self.status_callback,
            status_qos
        )
        
        # Periodic save timer
        self.periodic_timer = self.create_timer(self.periodic_interval, self.periodic_save)
        
        self.get_logger().info('Exploration Monitor started')
        self.get_logger().info(f'Periodic saves every {self.periodic_interval}s')
        self.get_logger().info('Monitoring /explore/status for completion')
    
    def config_callback(self, msg):
        """Receive world and trial ID from results_manager"""
        if self.world_id is None:  # Only log once
            parts = msg.data.split(',')
            self.world_id = parts[0]
            self.trial_id = parts[1]
            self.get_logger().info(f'Received config: World={self.world_id}, Trial={self.trial_id}')
    
    def status_callback(self, msg):
        """Handle exploration status updates"""
        self.get_logger().info(f'Exploration status: {msg.status}')
        
        # Save final map when robot returns to origin
        if msg.status == "returned_to_origin" and not self.final_save_done:
            self.get_logger().info('ROBOT RETURNED TO ORIGIN - SAVING FINAL MAP')
            self.save_map(final=True)
    
    def periodic_save(self):
        """Periodically save map during exploration"""
        if self.world_id is None or self.trial_id is None:
            return
        
        # Stop periodic saves after final save
        if self.final_save_done:
            return
        
        self.save_count += 1
        self.get_logger().info(f'Periodic save #{self.save_count}...')
        self.save_map(final=False)
    
    def save_map(self, final=False):
        """Save map using same naming convention as results_manager"""
        # Wait for config if not received yet
        if self.world_id is None or self.trial_id is None:
            self.get_logger().warn('No trial config received yet, skipping save...')
            return
        
        # Setup directory structure matching results_manager
        maps_dir = f'results/maps/World_{self.world_id}'
        os.makedirs(maps_dir, exist_ok=True)
        
        # Map filename matching results_manager convention
        map_name = f'trial_{self.trial_id}'
        map_path = os.path.join(maps_dir, map_name)
        
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path],
                capture_output=True,
                text=True,
                timeout=10.0
            )
            
            if result.returncode == 0:
                if final:
                    self.get_logger().info('FINAL MAP SAVED!')
                    self.get_logger().info(f'Location: {map_path}.yaml / {map_path}.pgm')
                    self.final_save_done = True
                else:
                    self.get_logger().info(f'Map saved: {map_path}.yaml')
            else:
                self.get_logger().error(f'Map save failed: {result.stderr}')
                
        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timed out')
        except Exception as e:
            self.get_logger().error(f'Error saving map: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()