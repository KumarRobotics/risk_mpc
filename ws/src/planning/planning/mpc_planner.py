import rclpy
import casadi
import numpy as np
import time

from rclpy.publisher import Publisher
from rclpy.node import Node
from casadi import DM, MX, Function
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from visualization_msgs.msg import Marker
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN
from typing import List, Tuple

class MPCTrajectoryPlanner(Node):
    def __init__(self) -> None:
        super().__init__('mpc_trajectory_planner')

        self.trajectory_publisher: Publisher[Path] = self.create_publisher(Path, '/planned_trajectory', 100)
        self.trajectory_points_publisher: Publisher[Marker] = self.create_publisher(Marker, '/path_points', 100)
        self.convex_hull_publisher: Publisher[Marker] = self.create_publisher(Marker, '/convex_hulls', 100)
        self.cluster_publisher: Publisher[Marker] = self.create_publisher(Marker, '/clusters', 100)
        self.occupied_points_publisher: Publisher[Marker] = self.create_publisher(Marker, '/occupied_points', 100)

        self.create_subscription(OccupancyGrid, '/cost_map', self.occupancy_grid_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.compute_trajectory_callback, 10)

        self.occupancy_grid: OccupancyGrid | None = None
        self.goal_pose: PoseStamped | None = None
        self.hulls = None

        self.get_logger().info("MPC Trajectory Planner Node Initialized")

    def occupancy_grid_callback(self, msg: OccupancyGrid) -> None:
        self.occupancy_grid = msg

    def compute_trajectory_callback(self, msg: PoseStamped) -> None:
        self.goal_pose = msg
        if self.occupancy_grid is None:
            self.get_logger().info("No Occupancy Grid Received")
            return

        # Process the occupancy grid to extract convex hulls representing obstacles
        hulls: List[np.ndarray[float]] = self.process_occupancy_grid_to_hulls()
        self.publish_convex_hulls(hulls)
        self.hulls = hulls

        goal: np.ndarray[float] = np.array([[self.goal_pose.pose.position.x, self.goal_pose.pose.position.y, 0.0]]).T
        control, states = self.solve_mpc(hulls, goal)

        if states is not None:
            self.get_logger().info("Trajectory computed successfully")
            self.publish_trajectory(states)
            self.publish_trajectory_points(states)
        else:    
            self.get_logger().info("Failed to compute trajectory.")

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

    def solve_mpc(self, hulls: List[np.ndarray[float]], goal: np.ndarray[float]):
        """
        Uses CasADi to solve the MPC optimization problem, avoiding convex hulls.
        """
        N: int = 300  # Prediction horizon
        dt: float = 0.05  # Time step

        x: MX = casadi.MX.sym('x')
        y: MX = casadi.MX.sym('y')
        theta: MX = casadi.MX.sym('theta')
        v: MX = casadi.MX.sym('v')
        omega: MX = casadi.MX.sym('omega')
        state: MX = casadi.vertcat(x, y, theta)
        control: MX = casadi.vertcat(v, omega)

        # Dynamics
        dynamics: DM = casadi.vertcat(
            v*casadi.cos(theta)*dt + x,
            v*casadi.sin(theta)*dt + y,
            omega*dt + theta
        )
        f: Function = casadi.Function('f', [state, control], [dynamics])

        X: MX = casadi.MX.sym('X', 3, N+1) 
        U: MX = casadi.MX.sym('U', 2, N)   
        Q: DM = casadi.diag([10, 10]) 
        R: DM = casadi.diag([1, 1/casadi.pi])  
        M: float = 10.0
        cost: MX = 0
        g: List[MX] = []

        # initial state constraint
        g.append(X[:, 0]) 
        
        # calculate cost expression and rollout dynamics constraints
        for k in range(N):
            # state and actuation cost
            cost += casadi.mtimes([(X[:2, k] - goal[:2]).T, Q, (X[:2, k] - goal[:2])])
            cost += casadi.mtimes([U[:, k].T, R, U[:, k]])

            # dynamics constraints
            x_next: MX = f(X[:, k], U[:, k])
            g.append(X[:, k + 1] - x_next)

        # final state constraint
        g.append(X[:2, -1] - goal[:2])
        dynamics_lower_bound: np.ndarray[float] = np.zeros(3*(N+2)-1)
        dynamics_upper_bound: np.ndarray[float] = np.zeros(3*(N+2)-1)

        # convex hull constraints
        for k in range(N+1):
            for hull in hulls:
                point_hull_constraints: List[MX] = []
                num_verticies: int = hull.shape[0]
                for i in range(num_verticies):
                    point_1: np.ndarray[float] = hull[i]
                    point_2: np.ndarray[float] = hull[(i+1) % num_verticies]

                    edge: np.ndarray[float] = point_2 - point_1
                    perp_vector: np.ndarray[float] = np.array([edge[1], -edge[0]])
                    normal: np.ndarray[float] = perp_vector / np.linalg.norm(perp_vector)
                    point_vec = X[:2, k] - point_1

                    comparison_value: MX = casadi.dot(normal, point_vec)
                    point_hull_constraints.append(comparison_value)
                g.append(casadi.mmax(casadi.vertcat(*point_hull_constraints)))
        hull_lower_bound: List[float] = [0]*(N+1)*len(hulls)
        hull_upper_bound: List[float] = [casadi.inf]*(N+1)*len(hulls)

        # input constraints
        for k in range(N):
            g.append(U[:, k])
        actuation_lower_bound: np.ndarray[float] = -np.tile((0, casadi.pi/4), N)
        actuation_upper_bound: np.ndarray[float] = np.tile((1, casadi.pi/4), N)

        nlp = {
            'x': casadi.vertcat(casadi.reshape(X, -1, 1), casadi.reshape(U, -1, 1)),
            'f': cost,
            'g': casadi.vertcat(*g)
        }

        solver = casadi.nlpsol('solver', 'ipopt', nlp)
        lower_bound = casadi.vertcat(dynamics_lower_bound, hull_lower_bound, actuation_lower_bound)
        upper_bound = casadi.vertcat(dynamics_upper_bound, hull_upper_bound, actuation_upper_bound)
        x_guess: np.ndarray[float] = np.linspace(np.zeros((3, 1)), goal, N+1).squeeze().T
        u_guess: np.ndarray[float] = np.zeros((2, N))
        sol = solver(lbg=lower_bound, ubg=upper_bound, x0=casadi.vertcat(x_guess.reshape(-1, 1), u_guess.reshape(-1, 1)))

        X_opt = casadi.reshape(sol['x'][:3 * (N + 1)], 3, N + 1)
        U_opt = casadi.reshape(sol['x'][3 * (N + 1):], 2, N)

        return U_opt.full(), X_opt.full()
    
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
        Publishes the convex hulls 
        """
        for i, hull in enumerate(hulls):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "convex_hulls"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            for row in hull:
                point = Point()
                point.x = row[0]
                point.y = row[1]
                point.z = 0.05
                marker.points.append(point)
            if len(hull) > 0:
                marker.points.append(Point(x=hull[0][0], y=hull[0][1], z=0.05))

            # Set marker properties
            marker.scale.x = 0.1  
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0  

            self.convex_hull_publisher.publish(marker)
            time.sleep(0.01)

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
    mpc_planner: MPCTrajectoryPlanner = MPCTrajectoryPlanner()
    rclpy.spin(mpc_planner)
    mpc_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
