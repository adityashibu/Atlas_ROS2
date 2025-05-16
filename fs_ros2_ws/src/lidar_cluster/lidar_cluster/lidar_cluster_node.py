#!/usr/bin/env python3
"""
Formula Student Driverless Project (FSD-Project).
Copyright (c) 2019-2025:
 - Original C++ by chentairan <killasipilin@gmail.com>
 - Adapted to Python ROS2

FSD-Project is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

FSD-Project is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with FSD-Project.  If not, see <https://www.gnu.org/licenses/>.
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from lidar_cluster.lidar_cluster import LidarCluster


class LidarClusterNode(Node):
    """ROS2 Node for LiDAR Clustering."""

    def __init__(self):
        """Initialize the LidarClusterNode."""
        super().__init__('lidar_cluster_node')
        
        self.get_logger().info('Initializing LidarClusterNode')
        
        # Load parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('raw_lidar_topic_name', '/carla/ego_vehicle/lidar'),
                ('lidar_cluster_topic_name', '/perception/lidar_cluster'),
                ('node_rate', 10)
            ])
            
        self.raw_lidar_topic = self.get_parameter('raw_lidar_topic_name').value
        self.cluster_topic = self.get_parameter('lidar_cluster_topic_name').value
        self.node_rate = self.get_parameter('node_rate').value
        
        self.get_logger().info(f'Raw lidar topic: {self.raw_lidar_topic}')
        self.get_logger().info(f'Cluster topic: {self.cluster_topic}')
        self.get_logger().info(f'Node rate: {self.node_rate}')
        
        # Create QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create LidarCluster processor
        self.lidar_cluster = LidarCluster(self)
        
        # Create subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            self.raw_lidar_topic,
            self.lidar_callback,
            qos)
            
        # Create publisher
        self.publisher = self.create_publisher(
            PointCloud,
            self.cluster_topic,
            qos)
            
        # Create timer
        self.timer = self.create_timer(1.0/float(self.node_rate), self.timer_callback)
        
        self.get_logger().info('LidarClusterNode initialized')
        
    def lidar_callback(self, msg):
        """Handle incoming LiDAR data."""
        self.lidar_cluster.set_raw_lidar(msg)
        
    def timer_callback(self):
        """Process LiDAR data at the specified rate."""
        start_time = time.time()
        
        self.lidar_cluster.run_algorithm()
        
        processing_time = time.time() - start_time
        self.get_logger().debug(f'Processing time: {processing_time:.6f} seconds')
        
        if self.lidar_cluster.is_ok():
            self.publisher.publish(self.lidar_cluster.get_lidar_cluster())
            

def main(args=None):
    """Run the LidarClusterNode."""
    rclpy.init(args=args)
    node = LidarClusterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()