#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.publisher import Publisher
from rclpy.node import Node
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Point, Polygon, Point32
from custom_msgs_pkg.msg import PolygonArray
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN
from typing import List, Tuple

class OccupancyGridParser(Node):
    def __init__(self) -> None:
        super().__init__('convex_hull_extractor')

        self.convex_hull_viz_publisher: Publisher[MarkerArray] = self.create_publisher(MarkerArray, '/convex_hulls_viz', 1)
        self.convex_hull_publisher: Publisher[PolygonArray] = self.create_publisher(PolygonArray, '/convex_hulls', 1)

        # self.create_subscription(OccupancyGrid, '/cost_map', self.occupancy_grid_callback, 10)
        self.create_subscription(OccupancyGrid, '/planners/dialted_occupancy_grid', self.occupancy_grid_callback, 10)

        self.occupancy_grid: OccupancyGrid | None = None
        self.hulls: List[np.ndarray[float]] | None = None

        self.get_logger().info("Occupancy Grid Parser Node Initialized")

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        """
        Generates convex hulls around the obstacles in the occupancy grid.
        """
        self.occupancy_grid = msg

        grid_info: MapMetaData = self.occupancy_grid.info
        width: int = grid_info.width
        height: int = grid_info.height
        resolution: float = grid_info.resolution
        origin: Tuple[float] = (grid_info.origin.position.x, grid_info.origin.position.y)
        data: np.ndarray[int] = np.array(self.occupancy_grid.data).reshape((height, width))

        # Extract occupied cells
        occupied_points: List[Tuple[float]] = []
        for i in range(height):
            for j in range(width):
                if data[i, j] == 100:  # Threshold for occupancy
                    x: float = origin[0] + j*resolution
                    y: float = origin[1] + i*resolution
                    occupied_points.append((x, y))

        # Group occupied cells into clusters (simple grid-based clustering)
        occupied_points: np.ndarray[float] = np.array(occupied_points)
        if len(occupied_points) == 0:
            return []

        clusters: List[np.ndarray[float]] = self.cluster_points(occupied_points, resolution)
        self.hulls = [self.compute_convex_hull(cluster) for cluster in clusters]

        self.get_logger().info(f"Generated {len(self.hulls)} convex hull obstacles.")
        self.publish_convex_hulls()

    def cluster_points(self, points: np.ndarray[float], resolution: float) -> List[np.ndarray[float]]:
        """
        Clusters points based on proximity using DBSCAN.
        """
        clustering: DBSCAN = DBSCAN(eps=5*resolution, min_samples=2).fit(points)
        labels: List[int] = clustering.labels_

        clusters: List[np.ndarray[float]] = []
        for label in set(labels):
            if label != -1:  # -1 indicates noise in DBSCAN
                clusters.append(points[labels == label])
        return clusters

    def compute_convex_hull(self, cluster: np.ndarray[float]) -> np.ndarray[float]:
        """
        Computes the convex hull of a cluster of points. Each hull is represented by its vertices
        in a counterclockwise order
        """
        # cannot make a hull out of a cluster of two points
        if len(cluster) < 3:
            return cluster  
        hull: ConvexHull = ConvexHull(cluster)
        return cluster[hull.vertices]
    
    def publish_convex_hulls(self) -> None:
        """
        Publishes the convex hulls to both RVIZ and as a polygon array
        """
        convex_hulls: PolygonArray = PolygonArray()
        marker_array: MarkerArray = MarkerArray()

        for i, hull in enumerate(self.hulls):
            # data structure for publishing to other nodes
            polygon: Polygon = Polygon()

            # data structure for RVIZ visualization
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "convex_hulls"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            # iterate over each point in the hull and add to both topics
            for row in hull:
                point32: Point32 = Point32()
                point32.x = row[0]
                point32.y = row[1]
                polygon.points.append(point32)
            
                point: Point = Point()
                point.x = row[0]
                point.y = row[1]
                point.z = 0.05
                marker.points.append(point)
            convex_hulls.polygons.append(polygon)
            
            # close the hull for visualization
            if len(hull) > 0:
                marker.points.append(Point(x=hull[0][0], y=hull[0][1], z=0.05))

            # Set marker properties
            marker.scale.x = 0.1  
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  

            marker_array.markers.append(marker) 

        self.convex_hull_viz_publisher.publish(marker_array)
        self.convex_hull_publisher.publish(convex_hulls)

def main(args=None):
    rclpy.init(args=args)
    mpc_planner: OccupancyGridParser = OccupancyGridParser()
    rclpy.spin(mpc_planner)
    mpc_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
