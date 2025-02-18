#!/usr/bin/env python3

import cv2
import rclpy
import heapq
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped

from typing import Dict, Set, Tuple, List

SQRT2 = 2**0.5

class JPSPlanner(Node):
    def __init__(self) -> None:
        super().__init__('jps_planner')
        self.create_subscription(Odometry, '/Odometry', self.odometry_callback, 10)
        self.create_subscription(OccupancyGrid, '/llm_planning_node/filtered_costmap', self.occupancy_grid_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_publisher = self.create_publisher(Path, '/planners/jump_point_path', 10)
        self.dialted_occupancy_grid_publisher = self.create_publisher(OccupancyGrid, '/planners/dialted_occupancy_grid', 10)

        self.odometry: Odometry | None = None
        self.occupancy_grid: OccupancyGrid | None = None
        self.resolution: float | None = None
        self.width: int | None = None
        self.height: int | None = None
        self.resolution: float | None = None

        self.origin: Tuple[float, float] | None = None
        self.goal: PoseStamped | None = None

        self.dialation: float = 0.5

        self.directions: List[Tuple[int, int]] = [
            (1, 0), (-1, 0), (0, 1), (0, -1), 
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

        self.get_logger().info("JPS Planner Node Initialized")

    def odometry_callback(self, msg: Odometry) -> None:
        """Processes Odometry messages from ROS 2"""
        self.odometry = msg

        
    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        """Processes OccupancyGrid messages from ROS 2"""
        self.occupancy_grid: OccupancyGrid = msg
        self.width: int = msg.info.width
        self.height: int = msg.info.height
        self.resolution: float = msg.info.resolution
        self.origin: Tuple[float] = (msg.info.origin.position.x, msg.info.origin.position.y)
        data: np.ndarray[float] = np.array(msg.data).reshape((self.height, self.width))

        expansion_pixels: int = int(np.ceil(self.dialation / self.resolution))
        obstacle_mask: np.ndarray = (data == 100).astype(np.uint8)
        kernel: np.ndarray = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * expansion_pixels + 1, 2 * expansion_pixels + 1))
        dilated_mask: np.ndarray = cv2.dilate(obstacle_mask, kernel)
        self.data: np.ndarray = data.copy()
        self.data[dilated_mask == 1] = 100

        self.dialated_grid: OccupancyGrid = OccupancyGrid()
        self.dialated_grid.header = self.occupancy_grid.header
        self.dialated_grid.info = self.occupancy_grid.info
        self.dialated_grid.data = self.data.astype(np.int8).flatten().tolist()
        self.dialted_occupancy_grid_publisher.publish(self.dialated_grid)

    def meters_to_grid(self, x_meters: float, y_meters: float) -> Tuple[int, int]:
        """
        Convert points in meter space to points in grid index space
        """
        grid_x: int = int((x_meters - self.origin[0]) / self.resolution)
        grid_y: int = int((y_meters - self.origin[1]) / self.resolution)
        return grid_x, grid_y

    def goal_callback(self, msg: PoseStamped) -> None:
        """Receives goal position and triggers path planning"""
        if self.occupancy_grid is None:
            self.get_logger().warn("No Occupancy Grid Received yet.")
            return
        
        if self.odometry is None:
            self.get_logger().warn("No Odometry Received yet.")
            return
        
        # convert start to grid coordinates
        self.get_logger().info(f"Odometry: {self.odometry.pose.pose.position}")
        sx, sy = self.meters_to_grid(self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y)

        # Convert goal to grid coordinates
        gx, gy = self.meters_to_grid(msg.pose.position.x, msg.pose.position.y)

        start: Tuple[int, int] = (sy, sx)
        goal: Tuple[int, int] = (gy, gx)

        self.get_logger().info(f"Set start at ({sx}, {sy}) in grid coordinates.")
        self.get_logger().info(f"Received goal at ({gx}, {gy}) in grid coordinates.")

        # Run JPS Algorithm
        path = self.run_jps(start, goal)

        # Publish path if found
        if path:
            self.publish_path(path)

    def is_valid(self, x: int, y: int) -> bool:
        """Checks if a position is within bounds and not an obstacle"""
        if 0 <= x < self.height and 0 <= y < self.width:
            return self.data[x, y] == 0
        return False

    def jump(self, x: int, y: int, dx: int, dy: int, goal: Tuple[int]) -> Tuple[int, int]:
        """Jump function to identify jump points"""
        if not self.is_valid(x, y):
            return None
        if (x, y) == goal:
            return x, y

        # Forced neighbor check (important for JPS)
        if dx != 0 and dy != 0:  # Diagonal
            if (self.is_valid(x - dx, y) and not self.is_valid(x - dx, y + dy)) or \
                (self.is_valid(x, y - dy) and not self.is_valid(x + dx, y - dy)):
                return x, y
        else:  # Straight
            if dx != 0:
                if (self.is_valid(x + dx, y + 1) and not self.is_valid(x, y + 1)) or \
                    (self.is_valid(x + dx, y - 1) and not self.is_valid(x, y - 1)):
                    return x, y
            else:
                if (self.is_valid(x + 1, y + dy) and not self.is_valid(x + 1, y)) or \
                    (self.is_valid(x - 1, y + dy) and not self.is_valid(x - 1, y)):
                    return x, y
                
        # Diagonal recursive jump check
        if dx != 0 and dy != 0:
            if self.jump(x + dx, y, dx, 0, goal) or self.jump(x, y + dy, 0, dy, goal):
                return x, y

        return self.jump(x + dx, y + dy, dx, dy, goal)

    def identify_successors(self, current_node: Tuple[int, int], goal: Tuple[int, int]):
        """Find successors using jump points"""
        x, y, g = current_node
        successors = []

        for dx, dy in self.directions:
            jump_point = self.jump(x + dx, y + dy, dx, dy, goal)
            if jump_point:
                jx, jy = jump_point
                cost = g + (SQRT2 if dx != 0 and dy != 0 else 1)
                successors.append((jx, jy, cost))

        return successors

    def run_jps(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Executes Jump Point Search Algorithm"""
        open_list = []
        heapq.heappush(open_list, (0, start[0], start[1], 0))  # (f, x, y, g)
        came_from: Set[Tuple[int, int]] = {}
        g_score = {start: 0}

        while open_list:
            _, x, y, g = heapq.heappop(open_list)
            if (x, y) == goal:
                return self.reconstruct_path(came_from, goal)

            for successor in self.identify_successors((x, y, g), goal):
                sx, sy, new_g = successor
                if (sx, sy) not in g_score or new_g < g_score[(sx, sy)]:
                    g_score[(sx, sy)] = new_g
                    f = new_g + self.heuristic((sx, sy), goal)
                    heapq.heappush(open_list, (f, sx, sy, new_g))
                    came_from[(sx, sy)] = (x, y)

        self.get_logger().warn("No path found.")
        return []

    def reconstruct_path(self, came_from: Dict[Tuple[int, int], Tuple[int, int]], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstructs the path from the goal to the start"""
        path = []
        current = goal
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        return path[::-1]

    def publish_path(self, path) -> None:
        """Publishes the computed path as a ROS 2 Path message"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (gy, gx) in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = gx * self.resolution + self.origin[0]
            pose.pose.position.y = gy * self.resolution + self.origin[1]
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info("Published planned path.")

    def heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Euclidean heuristic for A*"""
        return ((node[0] - goal[0])**2 + (node[1] - goal[1])**2)**0.5


def main():
    rclpy.init()
    node: JPSPlanner = JPSPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
