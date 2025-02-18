import rclpy
import casadi
import numpy as np
import time

from rclpy.publisher import Publisher
from rclpy.node import Node
from casadi import DM, MX, Function
from nav_msgs.msg import MapMetaData, OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Polygon
from std_msgs.msg import Header
from custom_msgs_pkg.msg import PolygonArray
from visualization_msgs.msg import Marker
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN
from typing import List, Tuple

class MPCTrajectoryPlanner(Node):
    def __init__(self) -> None:
        super().__init__('mpc_trajectory_planner_approx')

        self.trajectory_publisher: Publisher[Path] = self.create_publisher(Path, '/planned_trajectory', 100)
        self.trajectory_points_publisher: Publisher[Marker] = self.create_publisher(Marker, '/path_points', 100)

        self.create_subscription(PoseStamped, '/current_pose', self.pose_callback, 10)
        self.create_subscription(PolygonArray, '/convex_hulls', self.convex_hulls_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.compute_trajectory_callback, 10)

        self.occupancy_grid: OccupancyGrid | None = None
        self.current_post: PoseStamped | None = None
        self.convex_hulls: PolygonArray | None = None
        self.goal_pose: PoseStamped | None = None

        self.get_logger().info("MPC Trajectory Planner Node Initialized")

    def pose_callback(self, msg: PoseStamped) -> None:
        self.current_pose = msg

    def convex_hulls_callback(self, msg: PolygonArray) -> None:
        self.convex_hulls = msg

    def compute_trajectory_callback(self, msg: PoseStamped) -> None:
        self.goal_pose = msg
        if self.convex_hulls is None:
            self.get_logger().info("No Convex Hulls Received")
            return
        
        if self.current_pose is None:
            self.get_logger().info("No Robot Pose Received")
            return
        
        # Extract current pose
        x_current: float = self.current_pose.pose.position.x
        y_current: float = self.current_pose.pose.position.y
        theta_current: float = self._yaw_from_quaternion(self.current_pose.pose.orientation)
        current_state: Tuple[float] = (x_current, y_current, theta_current)

        goal: np.ndarray[float] = np.array([[self.goal_pose.pose.position.x, self.goal_pose.pose.position.y, 0.0]]).T
        control, states = self.solve_mpc(current_state, goal)

        if states is not None:
            self.get_logger().info("Trajectory computed successfully")
            self.publish_trajectory(states)
        else:    
            self.get_logger().info("Failed to compute trajectory.")

    def solve_mpc(self, current_state: Tuple[float], goal: np.ndarray[float]):
        """
        Uses CasADi to solve the MPC optimization problem, avoiding convex hulls.
        """
        N: int = 200  # Prediction horizon
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
        beta: float = 20.0 # controls how well we approximate the max
        beta2: float = 25.0 # controls how fast reward from being away from obstacles decays
        lambda_1: float = 1000.0 # controls relative weight of obstacle avoidance with goal seeking
        cost: MX = 0
        g: List[MX] = []

        # initial state constraint
        g.append(X[:, 0] - current_state) 
        
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

        # convex hull costs
        for k in range(N+1):
            for polygon in self.convex_hulls.polygons:
                inner_sum: MX = 0
                num_verticies: int = len(polygon.points)
                for i in range(num_verticies):
                    point_1: np.ndarray[float] = np.array([polygon.points[i].x, polygon.points[i].y])
                    point_2: np.ndarray[float] = np.array([polygon.points[(i+1)%num_verticies].x, polygon.points[(i+1)%num_verticies].y])

                    edge: np.ndarray[float] = point_2 - point_1
                    perp_vector: np.ndarray[float] = np.array([edge[1], -edge[0]])
                    normal: np.ndarray[float] = perp_vector / np.linalg.norm(perp_vector)
                    point_vec = X[:2, k] - point_1

                    comparison_value: MX = casadi.dot(normal, point_vec)
                    # perform a convex approximation of the max function
                    inner_sum += casadi.exp(beta * comparison_value)
                max_approx = 1/beta * casadi.log(inner_sum)
                # negate when adding to cost so it can minimize this
                cost += lambda_1 * casadi.exp(-beta2 * max_approx) 

        # input constraints
        for k in range(N):
            g.append(U[:, k])
        actuation_lower_bound: np.ndarray[float] = -np.tile((1, casadi.pi/4), N)
        actuation_upper_bound: np.ndarray[float] = np.tile((1, casadi.pi/4), N)

        nlp = {
            'x': casadi.vertcat(casadi.reshape(X, -1, 1), casadi.reshape(U, -1, 1)),
            'f': cost,
            'g': casadi.vertcat(*g)
        }

        solver = casadi.nlpsol('solver', 'ipopt', nlp)
        lower_bound = casadi.vertcat(dynamics_lower_bound, actuation_lower_bound)
        upper_bound = casadi.vertcat(dynamics_upper_bound, actuation_upper_bound)
        x_guess: np.ndarray[float] = np.linspace(np.zeros((3, 1)), goal, N+1).squeeze().T
        u_guess: np.ndarray[float] = np.zeros((2, N))
        sol = solver(lbg=lower_bound, ubg=upper_bound, x0=casadi.vertcat(x_guess.reshape(-1, 1), u_guess.reshape(-1, 1)))

        X_opt = casadi.reshape(sol['x'][:3 * (N + 1)], 3, N + 1)
        U_opt = casadi.reshape(sol['x'][3 * (N + 1):], 2, N)

        return U_opt.full(), X_opt.full()

    def publish_trajectory(self, states):
        """
        Publishes the planned trajectory as a nav_msgs/Path message.
        """
        # data structure to publish as a continuous path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # data structure to publish individual path points
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = path_msg.header.stamp
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
            # add new point to the continuous path
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = states[0, i]
            pose.pose.position.y = states[1, i]
            pose.pose.orientation.z = np.sin(states[2, i] / 2)
            pose.pose.orientation.w = np.cos(states[2, i] / 2)
            path_msg.poses.append(pose)

            # add a new point to be published individually
            point = Point()
            point.x = states[0, i]
            point.y = states[1, i]
            point.z = 0.05
            marker.points.append(point)

        self.trajectory_publisher.publish(path_msg)
        self.trajectory_points_publisher.publish(marker)
        self.get_logger().info("Published planned trajectory.")

    def _yaw_from_quaternion(self, quat: Quaternion) -> float:
        return np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y**2 + quat.z**2))

def main(args=None):
    rclpy.init(args=args)
    mpc_planner: MPCTrajectoryPlanner = MPCTrajectoryPlanner()
    rclpy.spin(mpc_planner)
    mpc_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
