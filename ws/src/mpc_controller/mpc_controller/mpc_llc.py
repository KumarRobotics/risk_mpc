import rclpy
import casadi
import numpy as np

from rclpy.publisher import Publisher
from rclpy.node import Node
from casadi import DM, MX, Function
from geometry_msgs.msg import TwistStamped, Quaternion
from custom_msgs_pkg.msg import PolygonArray
from nav_msgs.msg import Path, Odometry
from typing import List, Tuple

class MPCController(Node):
    def __init__(self) -> None:
        super().__init__('mpc_controller')
        
        self.cmd_pub: Publisher[TwistStamped] = self.create_publisher(TwistStamped, '/cmd_vel', 1)

        self.create_subscription(Path, '/planned_trajectory', self.trajectory_callback, 10)
        self.create_subscription(Odometry, '/Odometry', self.pose_callback, 10)
        self.create_subscription(PolygonArray, '/convex_hulls', self.convex_hull_callback, 10)

        self.reference_trajectory: Path | None = None
        self.convex_hulls: PolygonArray | None = None
        self.current_pose: Odometry | None = None
        self.get_logger().info("MPC Controller Node Initialized")

    def trajectory_callback(self, msg: Path) -> None:
        self.reference_trajectory = msg

    def convex_hull_callback(self, msg: PolygonArray) -> None:
        self.convex_hulls = msg

    def pose_callback(self, msg: Odometry) -> None:
        self.current_pose = msg
        if self.reference_trajectory is not None:
            self.compute_control()

    def compute_control(self) -> None:
        # Extract current pose
        x_current: float = self.current_pose.pose.pose.position.x
        y_current: float = self.current_pose.pose.pose.position.y
        theta_current: float = self._yaw_from_quaternion(self.current_pose.pose.pose.orientation)
        current_state: Tuple[float] = (x_current, y_current, theta_current)

        # Extract reference trajectory points
        ref_points: np.ndarray[float] = np.array([
            (pose.pose.position.x, pose.pose.position.y, self._yaw_from_quaternion(pose.pose.orientation)) 
            for pose in self.reference_trajectory.poses])

        u_opt = self.solve_mpc(current_state, ref_points)

        # Publish the first control input
        cmd: TwistStamped = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = u_opt[0, 0].full().item()
        cmd.twist.angular.z = u_opt[1, 0].full().item()
        self.cmd_pub.publish(cmd)

    def solve_mpc(self, current_state: Tuple[float], ref_points: np.ndarray[float]):
        # Find nearest reference point
        distances: np.ndarray[float] = np.sum(np.square(ref_points[:, :2] - current_state[:2]), axis=1)
        nearest_index: int = np.argmin(distances)

        # Create MPC optimization
        N: int = 20
        dt: float = 0.05

        x = casadi.MX.sym('x')
        y = casadi.MX.sym('y')
        theta = casadi.MX.sym('theta')
        v = casadi.MX.sym('v')
        omega = casadi.MX.sym('omega')
        state = casadi.vertcat(x, y, theta)
        control = casadi.vertcat(v, omega)

        dynamics: DM = casadi.vertcat(
            v*casadi.cos(theta)*dt + x,
            v*casadi.sin(theta)*dt + y,
            omega*dt + theta
        )
        f: Function = casadi.Function('f', [state, control], [dynamics])

        X: MX = casadi.MX.sym('X', 3, N + 1) 
        U: MX = casadi.MX.sym('U', 2, N)     
        Q: DM = casadi.diag([5, 5]) 
        R: DM = casadi.diag([1, 1/casadi.pi])  
        beta: float = 20.0 # controls how well we approximate the max
        beta2: float = 25.0 # controls how fast reward from being away from obstacles decays
        lambda_1: float = 1000.0 # controls relative weight of obstacle avoidance with goal seeking
        cost: MX = 0
        g: List[MX] = []

        # initial state constraint
        g.append(X[:, 0] - current_state)

        for k in range(N):
            ref_state = ref_points[min(nearest_index + k, len(ref_points) - 1), :]

            cost += casadi.mtimes([(X[:2, k] - ref_state[:2]).T, Q, (X[:2, k] - ref_state[:2])])
            cost += casadi.mtimes([U[:, k].T, R, U[:, k]])

            x_next = f(X[:, k], U[:, k])
            g.append(X[:, k+1] - x_next)
        dynamics_lower_bound: np.ndarray[float] = np.zeros(3*(N+1))
        dynamics_upper_bound: np.ndarray[float] = np.zeros(3*(N+1))

        # add convex hull cost
        # for k in range(N+1):
        #     for polygon in self.convex_hulls.polygons:
        #         inner_sum: MX = 0
        #         num_verticies: int = len(polygon.points)
        #         for i in range(num_verticies):
        #             point_1: np.ndarray[float] = np.array([polygon.points[i].x, polygon.points[i].y])
        #             point_2: np.ndarray[float] = np.array([polygon.points[(i+1)%num_verticies].x, polygon.points[(i+1)%num_verticies].y])

        #             edge: np.ndarray[float] = point_2 - point_1
        #             perp_vector: np.ndarray[float] = np.array([edge[1], -edge[0]])
        #             normal: np.ndarray[float] = perp_vector / np.linalg.norm(perp_vector)
        #             point_vec = X[:2, k] - point_1

        #             comparison_value: MX = casadi.dot(normal, point_vec)
        #             # perform a convex approximation of the max function
        #             inner_sum += casadi.exp(beta * comparison_value)
        #         max_approx = 1/beta * casadi.log(inner_sum)
        #         # negate when adding to cost so it can minimize this
        #         cost += lambda_1 * casadi.exp(-beta2 * max_approx) 

        # input constraints
        for k in range(N):
            g.append(U[:, k])
        actuation_lower_bound: np.ndarray[float] = -np.tile((1, casadi.pi/4), N)
        actuation_upper_bound: np.ndarray[float] = np.tile((1, casadi.pi/4), N)

        nlp = {'x': casadi.vertcat(casadi.reshape(X, -1, 1), casadi.reshape(U, -1, 1)), 'f': cost, 'g': casadi.vertcat(*g)}
        solver = casadi.nlpsol('solver', 'ipopt', nlp)
        lower_bound = casadi.vertcat(dynamics_lower_bound, actuation_lower_bound)
        upper_bound = casadi.vertcat(dynamics_upper_bound, actuation_upper_bound)

        x_guess: np.ndarray[float] = np.zeros((3*(N + 1), 1))
        u_guess: np.ndarray[float] = np.zeros((2*N, 1))
        sol = solver(lbg=lower_bound, ubg=upper_bound, x0=casadi.vertcat(x_guess.reshape(-1, 1), u_guess.reshape(-1, 1)))
        u_opt = casadi.reshape(sol['x'][3 * (N + 1):], 2, N)
        return u_opt

    def _yaw_from_quaternion(self, quat: Quaternion) -> float:
        return np.arctan2(2.0 * (quat.w * quat.z + quat.x * quat.y), 1.0 - 2.0 * (quat.y**2 + quat.z**2))

def main(args=None):
    rclpy.init(args=args)
    mpc_controller = MPCController()
    rclpy.spin(mpc_controller)
    mpc_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
