import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point32
from typing import List, Set, Tuple
from itertools import chain

class NOORDetection(Node):
    def __init__(self) -> None:
        super().__init__('noor_detection_node')

        self.vertical_channels: int = 64
        self.horizontal_channels: int = 1024
        # self.lidar_height: float = 0.15481246512979946
        self.lidar_height: float = 0.4082

        self.vertical_angles: List[float] = [16.349001, 15.750999, 15.181001, 14.645001, 14.171, 13.595, 13.043, 12.509, 12.041, 11.471, 10.932, 10.401, 9.935001400000001, 9.3830004, 8.8369999, 8.3079996, 7.8410001, 7.2950006, 6.7539997, 6.2189999, 5.756, 5.2119999, 4.671, 4.1409998, 3.678, 3.1359999, 2.5969999, 2.0630002, 1.5980002, 1.061, 0.51800007, -0.011, -0.48699999, -1.0250001, -1.562, -2.102, -2.566, -3.105, -3.648, -4.1849999, -4.652, -5.1820002, -5.7289996, -6.2719998, -6.7420001, -7.2729998, -7.8119998, -8.362000500000001, -8.833000200000001, -9.3629999, -9.906999600000001, -10.46, -10.938, -11.473, -12.015, -12.581, -13.071, -13.604, -14.157001, -14.726998, -15.241, -15.773, -16.337999, -16.917999]
        self.vertical_angles: np.ndarray[float] = np.deg2rad(np.array(self.vertical_angles[::-1]))

        # Calculate horizontal and vertical distances
        self.rho_i: np.ndarray[float] = self.lidar_height / np.tan(-self.vertical_angles)
        self.vdl: np.ndarray[float] = np.abs(self.rho_i - np.roll(self.rho_i, 1))

        self.create_subscription(PointCloud2, '/ouster/points', self.point_cloud_callback, 10)

        self.last_pointcloud: PointCloud2 | None = None

        self.scanline_pub: Publisher[Marker] = self.create_publisher(Marker, '/scan_line_marker', 10)
        self.feature_points_pub: Publisher[Marker] = self.create_publisher(Marker, '/feature_points', 10)

        self.get_logger().info("NOORDetection initialized")

    def point_cloud_callback(self, msg) -> None:
        self.last_pointcloud = msg
        # self.noir_detect()
        self.noor_detect()

    def noir_detect(self) -> None:
        self.get_logger().info("Performing NOIR detection")

    def noor_detect(self) -> None:
        if self.last_pointcloud is None:
            self.get_logger().warning("No PointCloud2 data received yet.")
            return

        # Put points into traversable 3D array
        points = list(read_points(self.last_pointcloud, field_names=("x", "y", "z")))

        structured_array = np.array(points, dtype=[
            ('x', np.float32), 
            ('y', np.float32), 
            ('z', np.float32)
        ])
        # NOTE: row -1 is the beam closest to the vehicle
        point_array: np.ndarray[float] = np.stack([structured_array['x'], 
                                                    structured_array['y'], 
                                                    structured_array['z']], 
                                                    axis=-1).reshape((self.vertical_channels, self.horizontal_channels, 3))

        # visualize one angle as a sanity check on data organization
        # scan_lines: np.ndarray = point_array[:, 161:187, :]
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines)

        # scan_lines: np.ndarray = point_array[:, 212:237, :] ???????????????????????????/
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines)

        # scan_lines: np.ndarray = point_array[:, 338:358, :]
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines)

        # scan_lines: np.ndarray = point_array[:, 378:405, :]
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines)

        # scan_lines: np.ndarray = point_array[:, 639:662, :]
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines)

        # scan_lines: np.ndarray = point_array[:, 683:706, :]
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines)

        # scan_lines: np.ndarray = point_array[:, 805:825, :] ??????????????????????????????????
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines) 

        # scan_lines: np.ndarray = point_array[:, 849:883, :]
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines)

        # scan_lines: np.ndarray = point_array[:, 910:930, :] ????????????????????????????????????
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines)

        # scan_lines: np.ndarray = point_array[-1:, :, :]
        # self.publish_scan_line_marker(scan_lines)

        # TODO: set any points that are too close to the LIDAR to nan
        norms: np.ndarray[float] = np.linalg.norm(point_array, axis=2)
        point_array[norms < 0.5] = np.nan

        # the points that define the NOOR boundaries
        feature_points: Set[Tuple[float]] = set()

        column_ranges = chain(range(0, 161), range(187, 212), 
                                range(237, 338), range(358, 378), 
                                range(405, 639), range(662, 683), 
                                range(706, 805), range(825, 849), 
                                range(883, 910), range(930, self.horizontal_channels))

        # we perform the sweep check at each angle
        # for column_idx in range(self.horizontal_channels):
        for column_idx in column_ranges:
            last_valid_point: np.ndarray[float] | None = None

            # traverse each scan line and perform treshold checks
            for scan_line_idx in range(self.vertical_channels-1, -1, -1):
                current_point: np.ndarray[float] = point_array[scan_line_idx, column_idx]

                # skip over any points containing nan values
                if np.any(np.isnan(current_point)):
                    continue
                
                # hit a positive obstacle on this scan line, disregard rest of data
                if current_point[2] > 0.5:
                    break

                # initialize upon reaching the first valid point, or
                # if the current point is within the threshold distance, update last valid point
                if (last_valid_point is None or (np.linalg.norm(current_point[:2] - last_valid_point[:2]) < 2.0 * self.vdl[self.vertical_channels - scan_line_idx - 1])):
                    last_valid_point = current_point
                # exceeded threshold, mark this point as a feature point
                else:
                    feature_points.add(tuple(last_valid_point))
                    feature_points.add(tuple(current_point))
                    last_valid_point = current_point

        self.publish_feature_points(feature_points)

    def publish_scan_line_marker(self, scan_lines):
        marker: Marker = Marker()
        marker.header.frame_id = "os_sensor"  
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "scan_line"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05 
        marker.scale.y = 0.05
        marker.color.a = 1.0  
        marker.color.r = 0.0 
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Add points from scan_lines
        for angle_idx in range(scan_lines.shape[1]):
            for point_idx in range(scan_lines.shape[0]):
                x, y, z = scan_lines[point_idx, angle_idx, :]
                if not np.isnan(z):  # Ensure the point is valid
                    new_point = Point32()
                    new_point.x = float(x)
                    new_point.y = float(y)
                    new_point.z = float(z)
                    marker.points.append(new_point)
        
        self.scanline_pub.publish(marker)
    
    def publish_feature_points(self, feature_points: Set[Tuple[float]]):
        marker: Marker = Marker()
        marker.header.frame_id = "os_sensor" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "feature_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05  
        marker.scale.y = 0.05
        marker.color.a = 1.0 
        marker.color.r = 0.0  
        marker.color.g = 0.0
        marker.color.b = 1.0

        # Add points from scan_line_0
        for point in feature_points:
            x, y, z = point
            if not np.isnan(z):  # Ensure the point is valid
                new_point = Point32()
                new_point.x = float(x)
                new_point.y = float(y)
                new_point.z = float(z)
                marker.points.append(new_point)
        
        self.feature_points_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node: NOORDetection = NOORDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
