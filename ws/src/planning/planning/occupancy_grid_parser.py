import rclpy
import numpy as np
import time

from rclpy.publisher import Publisher
from rclpy.node import Node
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Polygon, Point32
from custom_msgs_pkg.msg import PolygonArray
from visualization_msgs.msg import Marker
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN
from typing import List, Tuple

class OccupancyGridParser(Node):
    def __init__(self) -> None:
        super().__init__('convex_hull_extractor')

        self.convex_hull_viz_publisher: Publisher[Marker] = self.create_publisher(Marker, '/convex_hulls_viz', 100)
        self.convex_hull_publisher: Publisher[PolygonArray] = self.create_publisher(PolygonArray, '/convex_hulls', 100)
        self.cluster_publisher: Publisher[Marker] = self.create_publisher(Marker, '/clusters', 100)
        self.occupied_points_publisher: Publisher[Marker] = self.create_publisher(Marker, '/occupied_points', 100)

        self.create_subscription(OccupancyGrid, '/cost_map', self.occupancy_grid_callback, 10)

        self.occupancy_grid: OccupancyGrid | None = None
        self.hulls = None

        self.get_logger().info("Occupancy Grid Parser Node Initialized")

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        self.occupancy_grid = msg
        self.parse_occupancy_grid()

    def parse_occupancy_grid(self) -> None:
        if self.occupancy_grid is None:
            self.get_logger().info("No Occupancy Grid Received Yet")
            return

        # Process the occupancy grid to extract convex hulls representing obstacles
        hulls: List[np.ndarray[float]] = self.process_occupancy_grid_to_hulls()
        self.hulls = hulls
        self.publish_convex_hulls(self.hulls)
        
    def process_occupancy_grid_to_hulls(self) -> List[np.ndarray[float]]:
        """
        Groups neighboring cells in the occupancy grid into clusters and computes convex hulls.
        """
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
                    y: float = origin[0] + i * resolution
                    x: float = origin[1] + j * resolution
                    occupied_points.append((x, y))

        # Group occupied cells into clusters (simple grid-based clustering)
        occupied_points: np.ndarray[float] = np.array(occupied_points)
        if len(occupied_points) == 0:
            return []

        clusters: List[np.ndarray[float]] = self.cluster_points(occupied_points, resolution)
        hulls: List[np.ndarray[float]] = [self.compute_convex_hull(cluster) for cluster in clusters]

        self.get_logger().info(f"Generated {len(hulls)} convex hull obstacles.")
        return hulls

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
    
    def publish_occupied_points(self, occupied_points) -> None:
        for i, row in enumerate(occupied_points):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "occupied_points"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD

            point = Point()
            point.x = row[0]
            point.y = row[1]
            point.z = 0.05
            marker.points.append(point)
        
            # Set marker properties
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0  # Alpha (transparency)

            self.occupied_points_publisher.publish(marker)
            time.sleep(0.01)
    
    def publish_clusters(self, clusters):
        """
        Publishes the cluseters
        """
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "clusters"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD

            for row in cluster:
                point = Point()
                point.x = row[0]
                point.y = row[1]
                point.z = 0.05
                marker.points.append(point)
            
            # Set marker properties
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            self.cluster_publisher.publish(marker)
            time.sleep(0.01)
    
    def publish_convex_hulls(self, hulls):
        """
        Publishes the convex hulls to both RVIZ and as a polygon array
        """
        convex_hulls: PolygonArray = PolygonArray()

        for i, hull in enumerate(hulls):
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

            self.convex_hull_viz_publisher.publish(marker)
            time.sleep(0.01)
        
        # publish the polygon array
        self.convex_hull_publisher.publish(convex_hulls)

    def publish_trajectory(self, states):
        """
        Publishes the planned trajectory as a nav_msgs/Path message.
        """
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(states.shape[1]):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = states[0, i]
            pose.pose.position.y = states[1, i]
            pose.pose.orientation.z = np.sin(states[2, i] / 2)
            pose.pose.orientation.w = np.cos(states[2, i] / 2)

            path_msg.poses.append(pose)

        self.trajectory_publisher.publish(path_msg)
        self.get_logger().info("Published planned trajectory.")

    def publish_trajectory_points(self, states):
        """
        Publishes the planned trajectory as a Marker
        """
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0  

        for i in range(states.shape[1]):
            point = Point()
            point.x = states[0, i]
            point.y = states[1, i]
            point.z = 0.05
            marker.points.append(point)

        for j in range(10):
            self.trajectory_points_publisher.publish(marker)
        self.get_logger().info("Published trajectory points.")

def main(args=None):
    rclpy.init(args=args)
    mpc_planner: OccupancyGridParser = OccupancyGridParser()
    rclpy.spin(mpc_planner)
    mpc_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
