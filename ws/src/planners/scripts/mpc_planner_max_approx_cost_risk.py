#!/usr/bin/env python3

import rclpy
import casadi
import numpy as np

from rclpy.publisher import Publisher
from rclpy.node import Node
from casadi import DM, MX, Function
from casadi import interpolant
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from custom_msgs_pkg.msg import PolygonArray
from tf2_ros import TransformException
from tf2_ros import Buffer, TransformListener, TransformStamped
from visualization_msgs.msg import Marker

from typing import List, Tuple

class MPCTrajectoryPlanner(Node):
    def __init__(self) -> None:
        super().__init__('mpc_trajectory_planner_approx_risk')

        self.trajectory_publisher: Publisher[Path] = self.create_publisher(Path, '/planning/planned_trajectory', 100)
        self.trajectory_points_publisher: Publisher[Marker] = self.create_publisher(Marker, '/planning/path_points', 100)

        self.create_subscription(Odometry, '/Odometry', self.odometry_callback, 10)
        self.create_subscription(PolygonArray, '/convex_hulls', self.convex_hulls_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.compute_trajectory_callback, 10)
        self.create_subscription(OccupancyGrid, '/risk_map', self.risk_map_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.get_transform)
        self.map_to_baselink: TransformStamped | None = None

        self.odometry: Odometry | None = None
        self.convex_hulls: PolygonArray | None = None
        self.goal_pose: PoseStamped | None = None

        self.risk_map: OccupancyGrid | None = None
        self.global_to_risk_center: np.ndarray[float] | None = None

        self.get_logger().info("MPC Trajectory Planner Node Initialized")

    def get_transform(self) -> None:
        try:
            from_frame = 'map'
            to_frame = 'base_link'
            self.map_to_baselink = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time())
        except (TransformException) as e:
            self.get_logger().error(f'Failed to get transform: {str(e)}')

    def odometry_callback(self, msg: Odometry) -> None:
        self.odometry = msg

    def convex_hulls_callback(self, msg: PolygonArray) -> None:
        self.convex_hulls = msg

    def risk_map_callback(self, msg: OccupancyGrid) -> None:
        self.risk_map = msg
        # position vector from global (0, 0) to lower left corner of local risk map
        global_to_lower_left: np.ndarray[float] = np.array([[self.risk_map.info.origin.position.x, 
                                                            self.risk_map.info.origin.position.y, 
                                                            0.0]]).T
        # position vector from lower left corner of local risk map to center of risk map
        lower_left_to_risk_center: np.ndarray[float] = np.array([[self.risk_map.info.height*self.risk_map.info.resolution/2,
                                                                self.risk_map.info.width*self.risk_map.info.resolution/2,
                                                                0.0]]).T
        self.global_to_risk_center = global_to_lower_left + lower_left_to_risk_center
        
    def compute_trajectory_callback(self, msg: PoseStamped) -> None:
        self.goal_pose = msg

        if self.convex_hulls is None:
            self.get_logger().info("No Convex Hulls Received")
            return
        
        if self.risk_map is None:
            self.get_logger().info("No Risk Map Received")
            return
        
        # will plan in robot frame, initialize x0 as (0, 0, yaw)
        current_state: Tuple[float] = (0.0, 0.0, self._yaw_from_quaternion(self.odometry.pose.pose.orientation))
        goal_map_frame: np.ndarray[float] = np.array([[self.goal_pose.pose.position.x, self.goal_pose.pose.position.y, 0.0]]).T
        goal_rel_risk_center: np.ndarray[float] = goal_map_frame - self.global_to_risk_center
        _, states = self.solve_mpc(current_state, goal_rel_risk_center)

        if states is not None:
            self.get_logger().info("Trajectory computed successfully")
            self.publish_trajectory(states)
        else:    
            self.get_logger().info("Failed to compute trajectory.")

    def solve_mpc(self, current_state: Tuple[float], goal: np.ndarray[float]) -> Tuple[MX]: # might be wrong datatype
        """
        Uses CasADi to solve the MPC optimization problem, avoiding convex hulls.

        Args:
            current_state (Tuple[float]): contains the position and orientation relative to the 
            local risk map. (0, 0, yaw_angle)
            goal: np.ndarray[float]: contains the goal relative to the local risk map (x_pos, y_pos)
        """
        self.get_logger().info(f"Start, Goal: {current_state, goal}")
        N: int = 150  # Prediction horizon
        dt: float = 0.1  # Time step

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
        dynamics_func: Function = casadi.Function('f', [state, control], [dynamics])

        X: MX = casadi.MX.sym('X', 3, N+1) 
        U: MX = casadi.MX.sym('U', 2, N)   
        Q: DM = casadi.diag([10, 10]) 
        R: DM = casadi.diag([1, 1/casadi.pi])  
        beta_max: float = 20.0 # controls how well we approximate the max
        beta_obs_decay: float = 500.0 # controls how fast reward from being away from obstacles decays
        lambda_obs: float = 1000.0 # controls relative weight of obstacle avoidance with goal seeking
        lambda_risk: float = 0.0 # controls the relative weight of risk avoidance
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
            x_next: MX = dynamics_func(X[:, k], U[:, k])
            g.append(X[:, k + 1] - x_next)

        # final state constraint
        g.append(X[:2, -1] - goal[:2])
        dynamics_lower_bound: np.ndarray[float] = np.zeros(3*(N+2)-1)
        dynamics_upper_bound: np.ndarray[float] = np.zeros(3*(N+2)-1)

        for polygon in self.convex_hulls.polygons:
            self.get_logger().info(f"Polygon:\n{polygon}")

        for k in range(N+1):
            for polygon in self.convex_hulls.polygons:
                inner_sum: MX = 0
                num_verticies: int = len(polygon.points)
                for i in range(num_verticies):
                    point_1: np.ndarray[float] = np.array([polygon.points[i].x, polygon.points[i].y])
                    point_2: np.ndarray[float] = np.array([polygon.points[(i+1)%num_verticies].x, polygon.points[(i+1)%num_verticies].y])
                    # convert these points into the local risk map frame
                    point_1_risk_frame = point_1 - self.global_to_risk_center[:2, 0]
                    point_2_risk_frame = point_2 - self.global_to_risk_center[:2, 0]

                    edge: np.ndarray[float] = point_2_risk_frame - point_1_risk_frame
                    perp_vector: np.ndarray[float] = np.array([edge[1], -edge[0]])
                    normal: np.ndarray[float] = perp_vector / np.linalg.norm(perp_vector)
                    point_vec = X[:2, k] - point_1

                    comparison_value: MX = casadi.dot(normal, point_vec)
                    # perform a convex approximation of the max function
                    inner_sum += casadi.exp(beta_max * comparison_value)
                max_approx = 1/beta_max * casadi.log(inner_sum)
                # negate when adding to cost so it can minimize this
                cost += lambda_obs * casadi.exp(-beta_obs_decay * max_approx)

        # generate linear interpolation of risk map
        height: int = self.risk_map.info.height
        width: int = self.risk_map.info.width
        resolution: float = self.risk_map.info.resolution
        risk_data: List[float] = self.risk_map.data
        x_min: float = self.risk_map.info.origin.position.x - self.global_to_risk_center[0, 0] 
        y_min: float = self.risk_map.info.origin.position.y - self.global_to_risk_center[1, 0]
        x_max: float = x_min + resolution * height
        y_max: float = y_min + resolution * width
        x_coords: np.ndarray[float] = np.linspace(x_min, x_max, height)
        y_coords: np.ndarray[float] = np.linspace(y_min, y_max, width)
        continuous_risk_map: Function = interpolant('risk_map', 'bspline', [x_coords, y_coords], risk_data)
        
        # calculate risk cost
        # for k in range(N+1):
        #     risk = continuous_risk_map(casadi.vertcat(X[0, k], X[1, k]))
        #     cost += lambda_risk * risk

        # input constraints
        for k in range(N):
            g.append(U[:, k])
        actuation_lower_bound: np.ndarray[float] = -np.tile((0, casadi.pi/4), N)
        actuation_upper_bound: np.ndarray[float] = np.tile((2, casadi.pi/4), N)

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
            pose.pose.position.x = states[0, i] + self.global_to_risk_center[0, 0]
            pose.pose.position.y = states[1, i] + self.global_to_risk_center[1, 0]
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
