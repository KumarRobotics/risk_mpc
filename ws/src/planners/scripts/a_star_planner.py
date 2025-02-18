#!/usr/bin/env python3

import rclpy
import numpy as np
from heapq import heappush, heappop
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Polygon
from custom_msgs_pkg.msg import PolygonArray

from typing import Dict, Tuple, List

class AStarPlanner(Node):
    def __init__(self) -> None:
        super().__init__('a_star_planner')
        self.create_subscription(Odometry, '/Odometry', self.odometry_callback, 10)
        self.create_subscription(OccupancyGrid, '/llm_planning_node/filtered_costmap', self.occupancy_grid_callback, 10)
        # self.create_subscription(PolygonArray, '/convex_hulls', self.convex_hulls_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.publisher_path = self.create_publisher(Path, '/planners/a_star_path', 10)

        self.odometry: Odometry | None = None
        self.occupancy_grid: OccupancyGrid | None = None
        self.resolution: float | None = None
        self.width: int | None = None
        self.height: int | None = None
        self.resolution: float | None = None

        self.origin: Tuple[float] | None = None
        self.goal: PoseStamped | None = None
        # self.convex_hulls_grid: List[Polygon] = None

        self.get_logger().info("A* Planner Node Initialized")

        self.directions: List[Tuple[int]] = [
            (1, 0), (-1, 0), (0, 1), (0, -1), 
            (1, 1), (1, -1), (-1, 1), (-1, -1)
        ]

    def odometry_callback(self, msg: Odometry) -> None:
        """Processes Odometry messages from ROS 2"""
        self.odometry = msg
        
    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        """Processes OccupancyGrid messages from ROS 2"""
        self.occupancy_grid: OccupancyGrid = msg
        self.width: int = msg.info.width
        self.height: int = msg.info.height
        self.resolution: float = msg.info.resolution
        self.origin: List[float] = [msg.info.origin.position.x, msg.info.origin.position.y]
        self.data: np.ndarray[float] = np.array(msg.data).reshape((self.height, self.width))

    def meters_to_grid(self, x_meters: float, y_meters: float) -> Tuple[int]:
        """
        Convert points in meter space to points in grid index space
        """
        grid_x: int = int((x_meters - self.origin[0]) / self.resolution)
        grid_y: int = int((y_meters - self.origin[1]) / self.resolution)
        return grid_x, grid_y

    def goal_callback(self, msg: PoseStamped):
        """Receives goal position and triggers path planning"""
        if self.occupancy_grid is None:
            self.get_logger().warn("No Occupancy Grid Received yet.")
            return
        
        if self.odometry is None:
            self.get_logger().warn("No Odometry Received yet.")
            return
        
        # if self.convex_hulls_grid is None:
        #     self.get_logger().warn("No Convex Hulls Received yet.")
        #     return
        
        # convert start to grid coordinates
        self.get_logger().info(f"Odometry: {self.odometry.pose.pose.position}")
        sx, sy = self.meters_to_grid(self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y)

        # Convert goal to grid coordinates
        gx, gy = self.meters_to_grid(msg.pose.position.x, msg.pose.position.y)

        start: Tuple[int, int] = (sy, sx)
        goal: Tuple[int, int] = (gy, gx)

        self.get_logger().info(f"Set start at ({sx}, {sy}) in grid coordinates.")
        self.get_logger().info(f"Received goal at ({gx}, {gy}) in grid coordinates.")

        path: List[Tuple[int, int]] = self.a_star(start, goal)
        if path:
            self.publish_path(path)

    def is_valid(self, x: int, y: int) -> bool:
        """Checks if a position is within bounds and not an obstacle"""

        if 0 <= x < self.height and 0 <= y < self.width:
            return self.data[x, y] == 0
        return False

    def get_neighbors(self, current_node: Tuple[int]) -> List[Tuple[int, int]]:
        """Get the neighbors of the current cell"""
        x, y = current_node
        neighbors = []

        for direction in self.directions:
            new_x: int = x + direction[0]
            new_y: int = y + direction[1]
            if self.is_valid(new_x, new_y):
                neighbors.append((new_x, new_y))

        return neighbors

    def a_star(self, start: Tuple[int, int], goal: Tuple[int, int]):
        """Executes A Star Search Algorithm"""
        priority_queue: List[Tuple[int, Tuple[int, int]]] = [(0, start)]
        parents: Dict[Tuple[int, int]] = {}
        seen = set()
        cost = {start: 0}

        while priority_queue:
            _, location = heappop(priority_queue)

            if location == goal:
                path = self.reconstruct_path(parents, location)
                return path
            
            for neighbor in self.get_neighbors(location):
                if neighbor not in seen:
                    # update cost to include distance to neighbor
                    new_cost: float = cost[location] + ((location[0] - neighbor[0])**2 + (location[1] - neighbor[1])**2)**0.5
                    # perform update if we have not seen this node or if we found a shorter path to this node
                    if neighbor not in cost or new_cost < cost[neighbor]:
                        cost[neighbor] = new_cost
                        # add distance to goal if we want to run astar
                        heuristic = new_cost + ((goal[0] - neighbor[0])**2 + (goal[1] - neighbor[1])**2)**0.5
                        self.get_logger().info(f"{priority_queue}")
                        self.get_logger().info(f"{(heuristic, neighbor)}")
                        # add the neighbor into the queue based on the heuristic value
                        heappush(priority_queue, (heuristic, neighbor))
                        # assign the parent pointer
                        parents[neighbor] = location

        self.get_logger().warn("No path found.")
        return []

    def reconstruct_path(self, came_from: Dict[Tuple[int, int], Tuple[int, int]], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """Reconstructs the path from the goal to the start"""
        path: List[Tuple[int, int]] = []
        current = goal
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.append(current)
        return path[::-1]

    def publish_path(self, path: List[Tuple[int, int]]):
        """Publishes the computed path as a ROS 2 Path message"""
        path_msg: Path = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for (gy, gx) in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = gx * self.resolution + self.origin[0]
            pose.pose.position.y = gy * self.resolution + self.origin[1]
            path_msg.poses.append(pose)

        self.publisher_path.publish(path_msg)
        self.get_logger().info("Published planned path.")

    def heuristic(self, node: Tuple[int, int], goal: Tuple[int, int]) -> float:
        """Euclidean heuristic for A*"""
        return ((node[0] - goal[0])**2 + (node[1] - goal[1])**2)**0.5


def main():
    rclpy.init()
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
