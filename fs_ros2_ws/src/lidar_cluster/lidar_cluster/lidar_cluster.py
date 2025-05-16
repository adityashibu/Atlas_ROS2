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

import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point32
import rclpy
from rclpy.node import Node


class LidarCluster:
    """LiDAR cluster processor for Formula Student cone detection."""

    def __init__(self, node: Node):
        """Initialize LidarCluster."""
        self.node = node
        self.get_raw_lidar = False
        self.is_ok_flag = False
        self.raw_pc2 = None
        self.cluster = PointCloud()
        self.cluster.header.frame_id = "base_link"

    def is_ok(self):
        """Check if cluster is ready."""
        return self.is_ok_flag

    def get_lidar_cluster(self):
        """Get the clustered point cloud."""
        return self.cluster

    def set_raw_lidar(self, msg: PointCloud2):
        """Set the raw LiDAR data."""
        self.raw_pc2 = msg
        self.get_raw_lidar = True

    def run_algorithm(self):
        """Run the clustering algorithm."""
        if not self.raw_pc2 or not self.get_raw_lidar:
            return

        self.get_raw_lidar = False

        # Convert ROS PointCloud2 to numpy array
        pc_data = list(pc2.read_points(self.raw_pc2, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        if not pc_data:
            return

        # Convert to numpy array
        points = np.array(pc_data)

        # Filter points
        filtered_points = self._filter_points(points)
        if filtered_points.shape[0] == 0:
            return

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(filtered_points[:, :3])

        # Segment ground plane
        ground_points, cone_points = self._segment_ground(pcd)

        # Perform clustering on remaining points
        clusters = self._cluster_points(cone_points)

        # Process clusters to get cone positions
        self._process_clusters(clusters)

        self.cluster.header.stamp = self.raw_pc2.header.stamp
        self.is_ok_flag = True

    def _filter_points(self, points):
        """Filter points based on distance and height criteria."""
        # Filter points based on criteria similar to the C++ code
        filtered_indices = []
        for i, point in enumerate(points):
            x, y, z = point[:3]
            dist_xy = np.hypot(x, y)
            
            # Skip points that are too close, too high, behind the car, or far away and low
            if (dist_xy < np.sqrt(2) or 
                z > 0.7 or 
                x < 0 or 
                (dist_xy > 7 and z < 0.03)):
                continue
                
            filtered_indices.append(i)
            
        return points[filtered_indices] if filtered_indices else np.array([])

    def _segment_ground(self, pcd):
        """Segment ground points from the point cloud."""
        # Use RANSAC to detect the ground plane
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.07,
                                               ransac_n=3,
                                               num_iterations=100)
        
        # Extract ground and non-ground points
        ground_cloud = pcd.select_by_index(inliers)
        non_ground_cloud = pcd.select_by_index(inliers, invert=True)
        
        return ground_cloud, non_ground_cloud

    def _cluster_points(self, pcd):
        """Perform Euclidean clustering on the point cloud."""
        # Use DBSCAN clustering
        clustering = pcd.cluster_dbscan(eps=0.5, min_points=2)
        
        # Group points by cluster label
        max_label = max(clustering) if clustering else -1
        clusters = []
        
        for i in range(max_label + 1):
            # Get points for this cluster
            cluster_indices = [j for j, label in enumerate(clustering) if label == i]
            
            # Skip if too many points (not likely to be a cone)
            if len(cluster_indices) > 200:
                continue
                
            cluster_pcd = pcd.select_by_index(cluster_indices)
            clusters.append(cluster_pcd)
            
        return clusters

    def _process_clusters(self, clusters):
        """Process clusters to identify cones."""
        # Clear previous cluster data
        self.cluster.points = []
        
        for cluster_pcd in clusters:
            # Skip clusters with too few points
            if len(cluster_pcd.points) < 2:
                continue
                
            # Get bounds
            min_bound = cluster_pcd.get_min_bound()
            max_bound = cluster_pcd.get_max_bound()
            
            # Calculate dimensions
            bound_x = abs(max_bound[0] - min_bound[0])
            bound_y = abs(max_bound[1] - min_bound[1])
            bound_z = abs(max_bound[2] - min_bound[2])
            
            # Get centroid
            centroid = cluster_pcd.get_center()
            
            # Filter based on the shape of cones (similar to the C++ code)
            if (bound_x < 0.5 and bound_y < 0.5 and 
                bound_z < 0.4 and centroid[2] < 0.4):
                # Add this point to the result
                point = Point32()
                point.x = float(centroid[0])
                point.y = float(centroid[1])
                point.z = float(centroid[2])
                self.cluster.points.append(point)