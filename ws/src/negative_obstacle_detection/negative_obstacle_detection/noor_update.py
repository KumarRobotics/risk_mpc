import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point32, Pose, Quaternion, Vector3

from tf2_ros import Buffer, TransformListener, TransformStamped
from tf2_ros import TransformException

from itertools import chain
from typing import List, Set, Tuple
from scipy.spatial.transform import Rotation
from concurrent.futures import ThreadPoolExecutor, as_completed

class NOORDetection(Node):
    def __init__(self) -> None:
        super().__init__('noor_confidence_node')

        self.vertical_channels: int = 64
        self.horizontal_channels: int = 1024
        # self.lidar_height: float = 0.15481246512979946
        self.lidar_height: float = 0.4082
        # TODO: query these values from the actual LIDAR
        self.vertical_angles: List[float] = [16.349001, 15.750999, 15.181001, 14.645001, 14.171, 13.595, 13.043,
                                            12.509, 12.041, 11.471, 10.932, 10.401, 9.935001400000001, 9.3830004,
                                            8.8369999, 8.3079996, 7.8410001, 7.2950006, 6.7539997, 6.2189999, 5.756,
                                            5.2119999, 4.671, 4.1409998, 3.678, 3.1359999, 2.5969999, 2.0630002,
                                            1.5980002, 1.061, 0.51800007, -0.011, -0.48699999, -1.0250001, -1.562,
                                            -2.102, -2.566, -3.105, -3.648, -4.1849999, -4.652, -5.1820002, -5.7289996,
                                            -6.2719998, -6.7420001, -7.2729998, -7.8119998, -8.362000500000001,
                                            -8.833000200000001, -9.3629999, -9.906999600000001, -10.46, -10.938, -11.473,
                                            -12.015, -12.581, -13.071, -13.604, -14.157001, -14.726998, -15.241, -15.773,
                                            -16.337999, -16.917999]
        self.vertical_angles = np.deg2rad(np.array(self.vertical_angles[::-1]))

        # calculate the expected distances between scan lines
        self.rho_i: np.ndarray = self.lidar_height / np.tan(-self.vertical_angles)
        self.vdl: np.ndarray = np.abs(self.rho_i - np.roll(self.rho_i, 1))

        self.create_subscription(PointCloud2, '/ouster/points', self.point_cloud_callback, 10)
        self.create_subscription(OccupancyGrid, '/planners/dialted_occupancy_grid', self.occupancy_grid_callback, 10)

        self.scanline_pub: Publisher[Marker] = self.create_publisher(Marker, '/scan_line_marker', 10)
        self.feature_points_pub: Publisher[Marker] = self.create_publisher(Marker, '/feature_points', 10)
        self.risk_map_pub: Publisher[OccupancyGrid] = self.create_publisher(OccupancyGrid, '/risk_map', 10)
        self.combined_map_pub: Publisher[OccupancyGrid] = self.create_publisher(OccupancyGrid, '/combined_map', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.get_transform)
        
        self.occupancy_grid: OccupancyGrid | None = None
        self.lidar_to_costmap_transform: TransformStamped | None = None
        self.last_pointcloud: PointCloud2 | None = None

        self.get_logger().info("NOORDetection initialized")

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        self.occupancy_grid = msg

    def get_transform(self) -> None:
        if self.occupancy_grid is None:
            return

        try:
            from_frame = 'os_sensor'
            to_frame = self.occupancy_grid.header.frame_id
            self.lidar_to_costmap_transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
        except (TransformException) as e:
            self.get_logger().error(f'Failed to get transform: {str(e)}')

    def point_cloud_callback(self, msg: PointCloud2) -> None:
        self.last_pointcloud = msg
        self.noor_detect()

    def noor_detect(self) -> None:
        if self.last_pointcloud is None:
            self.get_logger().warning("No PointCloud2 Data Received.")
            return

        if self.occupancy_grid is None:
            self.get_logger().warning("No Occupancy Grid Received.")
            return

        if self.lidar_to_costmap_transform is None:
            self.get_logger().warning("No Transform Received.")
            return

        points = list(read_points(self.last_pointcloud, field_names=("x", "y", "z")))

        structured_array = np.array(points, dtype=[
                                            ('x', np.float32), 
                                            ('y', np.float32), 
                                            ('z', np.float32)])
        # NOTE: row -1 is the beam closest to the vehicle
        point_array: np.ndarray[float] = np.stack([structured_array['x'], 
                                                structured_array['y'], 
                                                structured_array['z']], 
                                                axis=-1).reshape((self.vertical_channels, self.horizontal_channels, 3))

        # manually determine LIDAR-deadzones
        # scan_lines: np.ndarray = point_array[:, 161:187, :]
        # self.get_logger().info(f"Scan line 0 size: {scan_lines.shape}")
        # self.publish_scan_line_marker(scan_lines)

        # Set any points that are too close to the LIDAR to nan
        norms: np.ndarray[float] = np.linalg.norm(point_array, axis=2)
        point_array[norms < 0.5] = np.nan

        # the points that define the NOOR boundaries
        feature_points: Set[Tuple[Tuple[float], Tuple[float], float]] = set()

        column_ranges = chain(range(0, 161), range(187, 212), 
                            range(237, 338), range(358, 378), 
                            range(405, 639), range(662, 683), 
                            range(706, 805), range(825, 849), 
                            range(883, 910), range(930, self.horizontal_channels))
        dead_zones = chain(range(159, 188), range(210, 238),
                        range(336, 357), range(376, 406),
                        range(637, 663), range(681, 707),
                        range(803, 826), range(847, 884),
                        range(908, 931))

        # TODO: FIGURE OUT HOW THIS ACTUALLY WORKS BECAUSE I DON'T THINK IT'S WORKING
        with ThreadPoolExecutor() as executor:
            futures = {
                executor.submit(self.process_scan_column, point_array[:, col_idx, :]): col_idx
                for col_idx in column_ranges
            }

            for future in as_completed(futures):
                feature_points.update(future.result())

        self.publish_feature_points(feature_points)

        # extract relevant Occupancy Grid information
        grid_info: MapMetaData = self.occupancy_grid.info
        grid_origin: Pose = grid_info.origin
        width: int = grid_info.width
        height: int = grid_info.height
        resolution = grid_info.resolution

        # make risk occupancy grid and combined occupancy grid
        risk_grid: OccupancyGrid = OccupancyGrid()
        risk_grid.header = self.occupancy_grid.header
        risk_grid.info = grid_info
        risk_grid_data = [0]*height*width

        combined_grid: OccupancyGrid = OccupancyGrid()
        combined_grid.header = self.occupancy_grid.header
        combined_grid.info = grid_info
        combined_grid_data = self.occupancy_grid.data

        # get the transformation between the LIDAR frame and the costmap
        quat: Quaternion = self.lidar_to_costmap_transform.transform.rotation
        quat_array: List[float] = [quat.x, quat.y, quat.z, quat.w]
        rotation: Rotation = Rotation.from_quat(quat_array)
        translation: Vector3 = self.lidar_to_costmap_transform.transform.translation
        translation_array: np.ndarray[float] = np.array([translation.x, translation.y, translation.z])

        # use point correspondences to update the value of the occupancy grid
        for correspondence in feature_points:
            point1, point2, risk = correspondence

            # transform the points from the LIDAR frame to the costmap frame
            point1_cost_frame: np.ndarray[float] = rotation.apply(point1) + translation_array
            point2_cost_frame: np.ndarray[float] = rotation.apply(point2) + translation_array
            norm: float = np.linalg.norm(point1_cost_frame[:2] - point2_cost_frame[:2])

            # determine the 2D index in the cost_map data array
            j1: int = int((point1_cost_frame[0] - grid_origin.position.x)/resolution)
            i1: int = int((point1_cost_frame[1] - grid_origin.position.y)/resolution)
            j2: int = int((point2_cost_frame[0] - grid_origin.position.x)/resolution)
            i2: int = int((point2_cost_frame[1] - grid_origin.position.y)/resolution)

            # determine which cells would need to be updated using Brenseham's Algorithm
            grid_cells: List[Tuple[float]] = self.bresenham_line(i1, j1, i2, j2)
            for cell in grid_cells:
                cell_i: int = cell[0]
                cell_j: int = cell[1]
                # ensure the cell is within the grid
                if (0 <= cell_i < width) and (0 <= cell_j < height):
                    # populate the risk grid with the integrated risk
                    risk_grid_data[cell_i * height + cell_j] = min(int(norm*risk), 100)

                    # only update value in oriignal map if this location is not already an obstacle
                    if combined_grid_data[cell_i * height + cell_j] == 0:
                        combined_grid_data[cell_i * height + cell_j] = min(int(norm*risk), 100)

        # post-process dead regions by filling with adjacent risk data
        # for dead_col_idx in dead_zones:
        #     theta: float = 2*np.pi/1024 * dead_col_idx
        #     # determine which side of the cost_map the ray would intersect
        #     if 0 <= theta < np.pi/2:
        #         x_lim: int = -10
        #         y_lim: int = 10
        #     elif np.pi/2 <= theta < np.pi:
        #         x_lim: int = 10
        #         y_lim: int = 10
        #     elif np.pi <= theta < 3*np.pi/2:
        #         x_lim: int = 10
        #         y_lim: int = -10
        #     else:
        #         x_lim: int = -10
        #         y_lim: int = -10

        #     # calculate the position of the ray's intersection
        #     t_x: float = -x_lim*np.sqrt(2)/np.cos(theta)
        #     t_y: float = y_lim*np.sqrt(2)/np.sin(theta)
        #     t: float = min(t_x, t_y)
        #     grid_edge_point: Tuple[float] = (-t*np.cos(theta), t*np.sin(theta), 0)
        #     grid_edge_cost_frame: np.ndarray[float] = rotation.apply(grid_edge_point) + translation_array

        #     # extract the cells from the costmap center to the border
        #     j2: int = int((grid_edge_cost_frame[0] - grid_origin.position.x)/resolution)
        #     i2: int = int((grid_edge_cost_frame[1] - grid_origin.position.y)/resolution)
        #     grid_cells: List[Tuple[float]] = self.bresenham_line(50, 50, i2, j2)

        #     # populate risk value with neighboring risk value
        #     offset: int = -1 if theta <= np.pi else 1
        #     for cell in grid_cells:
        #         cell_i: int = cell[0]
        #         cell_j: int = cell[1]
        #         if (0 <= cell_i < width) and (0 <= cell_j + offset < height):
        #             # new_grid_data[cell_i * height + cell_j] = new_grid_data[cell_i * height + cell_j + offset]
        #             new_grid_data[cell_i * height + cell_j] = -1

        risk_grid.data = risk_grid_data
        combined_grid.data = combined_grid_data
        self.risk_map_pub.publish(risk_grid)
        self.combined_map_pub.publish(combined_grid)

    def process_scan_column(self, point_array_col: np.ndarray):
        feature_points: Set[Tuple[Tuple[float], float]] = set()
        last_valid_point: np.ndarray[float] | None = None

        for scan_line_idx in range(self.vertical_channels - 1, -1, -1):
            current_point: np.ndarray[float] = point_array_col[scan_line_idx]

            # skip over any points containing nan values
            if np.any(np.isnan(current_point)):
                continue

            # initialize first valid point
            if last_valid_point is None:
                last_valid_point = current_point
                continue

            # hit a positive obstacle on this scan line, disregard rest of data
            if current_point[2] > 0.5:
                break

            # point outside of costmap, disregard: TODO: MAKE THIS BETTER MAYBE
            if np.linalg.norm(current_point[:2]) >= 10*(2)**0.5:
                break

            expected_threshold: float = 2.0 * self.vdl[self.vertical_channels - scan_line_idx - 1]
            deviation: float = np.linalg.norm(current_point[:2] - last_valid_point[:2])
            # if the current point is within the threshold distance, update last valid point
            if (deviation < expected_threshold):
                last_valid_point = current_point
            # exceeded threshold, mark this point as a feature point and calculate deviation
            else:
                risk: float = deviation / expected_threshold
                feature_points.add((tuple(last_valid_point), tuple(current_point), risk))
                last_valid_point = current_point
        return feature_points

    def bresenham_line(self, i1: int, j1: int, i2: int, j2: int) -> List[Tuple[float]]:
        points: List[Tuple[float]] = []
        dx: int = abs(i2 - i1)
        dy: int = abs(j2 - j1)
        sx: int = 1 if i1 < i2 else -1
        sy: int = 1 if j1 < j2 else -1
        err: int = dx - dy

        while True:
            points.append((i1, j1))
            if i1 == i2 and j1 == j2:
                break
            e2: int = 2 * err
            if e2 > -dy:
                err -= dy
                i1 += sx
            if e2 < dx:
                err += dx
                j1 += sy
        return points

    def publish_scan_line_marker(self, scan_lines: np.ndarray[float]):
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

    def publish_feature_points(self, feature_points: Set[Tuple[Tuple[float], Tuple[float], float]]):
        marker: Marker = Marker()
        marker.header.frame_id = "os_sensor" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "feature_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05  
        marker.scale.y = 0.05

        # Add points from scan_line_0
        for point1, point2, risk in feature_points:
            x1, y1, z1 = point1
            x2, y2, z2 = point2

            new_point1 = Point32()
            new_point1.x = float(x1)
            new_point1.y = float(y1)
            new_point1.z = float(z1)
            marker.points.append(new_point1)

            new_point2 = Point32()
            new_point2.x = float(x2)
            new_point2.y = float(y2)
            new_point2.z = float(z2)
            marker.points.append(new_point2)

            color = ColorRGBA()
            color.a = 225.0
            color.r = risk 
            color.g = 225.0 - risk
            color.b = 0.0
            marker.colors.append(color)
            marker.colors.append(color)

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
