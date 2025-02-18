#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <custom_msgs_pkg/msg/polygon_array.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <visualization_msgs/visualization_msgs/msg/marker.hpp>
#include <decomp_ros_util/data_ros_utils.hpp>
#include <decomp_util/iterative_decomp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <casadi/casadi.hpp>
#include <chrono>

using namespace std;
using namespace casadi;

class MPCPlannerCorridors: public rclcpp::Node{
public:
    MPCPlannerCorridors(): Node("mpc_planner_corridors"){
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, bind(&MPCPlannerCorridors::odometryCallback, this, placeholders::_1));
        polygon_sub_ = this->create_subscription<custom_msgs_pkg::msg::PolygonArray>(
            "/convex_hulls", 10, bind(&MPCPlannerCorridors::polygonsCallback, this, placeholders::_1));
        risk_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/risk_map", 10, bind(&MPCPlannerCorridors::riskMapCallback, this, placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planners/jump_point_path", 10, bind(&MPCPlannerCorridors::pathCallback, this, placeholders::_1));
        // path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        //     "/planners/a_star_path", 10, bind(&MPCPlannerCorridors::pathCallback, this, placeholders::_1));
        travel_corridors_pub_ = this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(
            "/polyhedron_array", 10);
        mpc_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/planners/mpc_path", 10);
        mpc_path_points_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/planners/mpc_points", 10);
        RCLCPP_INFO(this->get_logger(), "Convex Travel Corridor Initialized.");
    }

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
        odometry_ = msg;
    }

    void polygonsCallback(const custom_msgs_pkg::msg::PolygonArray::SharedPtr msg) {
        for (const geometry_msgs::msg::Polygon& polygon: msg->polygons){
            for (const geometry_msgs::msg::Point32& point: polygon.points){
                obstacles_.push_back(Vec2f(point.x, point.y));
            }
        }
    }

    void riskMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        risk_map_ = msg;
        global_to_lower_left_ = reshape(DM({
                                            risk_map_->info.origin.position.x,
                                            risk_map_->info.origin.position.y,
                                            0.0}), 3, 1);
        lower_left_to_risk_center_ = reshape(DM({
                                            risk_map_->info.height*risk_map_->info.resolution/2,
                                            risk_map_->info.width*risk_map_->info.resolution/2,
                                            0.0}), 3, 1);
        global_risk_to_center_ = global_to_lower_left_ - lower_left_to_risk_center_;
        
        // vector<double> x_vector(risk_map_.info.width);
        // vector<double> y_vector(risk_map_.info.height);
        // for (size_t x = 0; x < risk_map_.info.width; ++x){
        //     x_vector[x] = risk_map_.info.origin.position.x + x*risk_map_.info.resolution;
        // } 
        // for (size_t y = 0; y < risk_map_.info.height; ++y){
        //     y_vector[y] = risk_map_.info.origin.position.y + y*risk_map_.info.resolution;
        // } 
        // vector<double> grid_coords;
        // grid_coords.insert(grid_coords.end(), x_vector.begin(), x_vector.end());
        // grid_coords.insert(grid_coords.end(), y_vector.begin(), y_vector.end());
        
        vector<double> risk_data_double(risk_map_->data.begin(), risk_map_->data.end());
        vector<casadi_int> grid_dims = {static_cast<casadi_int>(risk_map_->info.width), 
            static_cast<casadi_int>(risk_map_->info.height)};
        
        // need to get index position in order to query?
        // Function risk_continuous = interpolant("continuous_risk_map", "bspline", grid_dims, risk_data_double);
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!odometry_) {
            RCLCPP_WARN(this->get_logger(), "No Odometry Received Yet.");
            return;
        }

        if (!risk_map_) {
            RCLCPP_WARN(this->get_logger(), "No Risk Map Received Yet.");
            return;
        }

        vec_Vec2f path_2d;
        for (const geometry_msgs::msg::PoseStamped& pose : msg->poses) {
            path_2d.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }

        // generate cumulative proportion of path length metric
        vector<double> cum_distances;
        double total_distance = 0.0;
        for (unsigned int i=0; i<path_2d.size() - 1; ++i){
            Vec2f& point1 = path_2d[i];
            Vec2f& point2 = path_2d[i+1];
            double dist_squared = pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2);
            double dist = pow(dist_squared, 0.5);
            total_distance += dist;
            cum_distances.push_back(total_distance);
        }
        vector<double> path_proportions;
        double total_length = cum_distances.back();
        for (double& length: cum_distances){
            path_proportions.push_back(length / total_length);
        }

        IterativeDecomp2D decomp_util;
        decomp_util.set_obs(obstacles_);
        decomp_util.set_local_bbox(Vec2f(20, 20));
        decomp_util.dilate(path_2d);
        
        vec_E<Polyhedron2D> polyhedrons = decomp_util.get_polyhedrons();
        decomp_ros_msgs::msg::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(polyhedrons);
        poly_msg.header.frame_id = "map";
        travel_corridors_pub_->publish(poly_msg);
        RCLCPP_INFO(this->get_logger(), "Generated Travel Corridors");
        RCLCPP_INFO(this->get_logger(), "Generated %1ld Polygons", polyhedrons.size());
        RCLCPP_INFO(this->get_logger(), "Number of Segments: %1ld", path_2d.size() - 1);
        
        vector<double> goal_position = {msg->poses.back().pose.position.x, msg->poses.back().pose.position.y};
        generateTrajectory(polyhedrons, goal_position, path_proportions);
    }

    void generateTrajectory(vec_E<Polyhedron2D> polyhedrons, vector<double>& goal_position, vector<double>& path_proportions){
        const int N = 150;
        double dt = 0.1;

        SX X = SX::sym("state_variables", 3, N + 1);
        SX U = SX::sym("control_variables", 2, N);
        RCLCPP_INFO(this->get_logger(), "X size1: %1lld", X.size1());
        RCLCPP_INFO(this->get_logger(), "X size2: %1lld", X.size2());
        RCLCPP_INFO(this->get_logger(), "U size1: %1lld", U.size1());
        RCLCPP_INFO(this->get_logger(), "U size2: %1lld", U.size2());

        vector<SX> variables_list = {X, U};
        vector<string> variables_name = {"states", "inputs"};

        SX variables_flat = vertcat(reshape(X, X.size1()*X.size2(), 1), reshape(U, U.size1()*U.size2(), 1));

        Function pack_variables_fn = Function("pack_variables_fn", variables_list, {variables_flat}, variables_name, {"flat"});
        Function unpack_variables_fn = Function("unpack_variables_fn", {variables_flat}, variables_list, {"flat"}, variables_name);

        DMVector upper_bounds = unpack_variables_fn(DM::inf(variables_flat.rows(), 1));
        DMVector lower_bounds = unpack_variables_fn(-DM::inf(variables_flat.rows(), 1));
        RCLCPP_INFO(this->get_logger(), "Made Variables and Bound Arrays");

        // set initial and final state vectors
        SX initial_state = reshape(SX(vector<double>{
            odometry_->pose.pose.position.x, 
            odometry_->pose.pose.position.y, 
            yaw_from_quaternion(odometry_->pose.pose.orientation)}), 3, 1);
        SX final_position = reshape(SX(goal_position), 2, 1);
        RCLCPP_INFO(this->get_logger(), "Initial Pose: %s", initial_state.get_str().c_str());
        RCLCPP_INFO(this->get_logger(), "Final Pose: %s", final_position.get_str().c_str());

        // input bounds
        lower_bounds[1] = repmat(DM(vector<double>{0.0, -pi/4}), 1, lower_bounds[1].size2());
        upper_bounds[1] = repmat(DM(vector<double>{2.0, pi/4}), 1, upper_bounds[1].size2());
        RCLCPP_INFO(this->get_logger(), "Lower_Bounds[1].rows(): %1lld", lower_bounds[1].size1());
        RCLCPP_INFO(this->get_logger(), "Lower_Bounds[1].columns(): %1lld", lower_bounds[1].size2());
        RCLCPP_INFO(this->get_logger(), "Lower_Bounds[1](0, 0): %s", lower_bounds[1](0, 0).get_str().c_str());
        RCLCPP_INFO(this->get_logger(), "Lower_Bounds[1](1, 0): %s", lower_bounds[1](1, 0).get_str().c_str());
        RCLCPP_INFO(this->get_logger(), "Upper_Bounds[1].rows(): %1lld", upper_bounds[1].size1());
        RCLCPP_INFO(this->get_logger(), "Upper_Bounds[1].columns(): %1lld", upper_bounds[1].size2());
        RCLCPP_INFO(this->get_logger(), "Upper_Bounds[1](0, 0): %s", upper_bounds[1](0, 0).get_str().c_str());
        RCLCPP_INFO(this->get_logger(), "Upper_Bounds[1](1, 0): %s", upper_bounds[1](1, 0).get_str().c_str());
        RCLCPP_INFO(this->get_logger(), "Set Input Bounds");

        // state bounds
        lower_bounds[0](0, Slice()) = risk_map_->info.origin.position.x * DM::ones(1, lower_bounds[0].size2());
        lower_bounds[0](1, Slice()) = risk_map_->info.origin.position.y * DM::ones(1, lower_bounds[0].size2());
        upper_bounds[0](0, Slice()) = (risk_map_->info.origin.position.x + risk_map_->info.resolution * risk_map_->info.height
                                        ) * DM::ones(1, lower_bounds[0].size2());
        upper_bounds[0](1, Slice()) = (risk_map_->info.origin.position.y + risk_map_->info.resolution * risk_map_->info.width
                                        ) * DM::ones(1, lower_bounds[0].size2());

        // running state cost, control cost, and risk cost
        vector<vector<double>> Q_vals = {{10, 0}, {0, 10}};
        SX Q = SX(Q_vals);
        vector<vector<double>> R_vals = {{1, 0}, {0, 1/pi}};
        SX R = SX(R_vals);
        RCLCPP_INFO(this->get_logger(), "Sanity Check: Q: %s", Q.get_str().c_str());
        RCLCPP_INFO(this->get_logger(), "Sanity Check: R: %s", R.get_str().c_str());
        SX objective = 0.0;
        for (int k=0; k < N; ++k){
            SX state_penalty = X(Slice(0, 2), k) - final_position;
            // SX state_penalty = X(Slice(0, 2), k) - X(Slice(0, 2), k+1);
            SX control_penalty = U(Slice(), k);
            objective = objective + mtimes(mtimes(state_penalty.T(), Q), state_penalty);
            objective = objective + mtimes(mtimes(control_penalty.T(), R), control_penalty);
        }   
        RCLCPP_INFO(this->get_logger(), "Sanity Check: X(Slice(0, 2), 3) size1: %1lld", X(Slice(0, 2), 3).size1());
        RCLCPP_INFO(this->get_logger(), "Sanity Check: X(Slice(0, 2), 3) size2: %1lld", X(Slice(0, 2), 3).size2());
        RCLCPP_INFO(this->get_logger(), "Sanity Check: U(Slice(), 3) size1: %1lld", U(Slice(), 3).size1());
        RCLCPP_INFO(this->get_logger(), "Sanity Check: U(Slice(), 3) size2: %1lld", U(Slice(), 3).size2());
        RCLCPP_INFO(this->get_logger(), "Sanity Check: Objective: %s", objective.get_str().c_str());
        RCLCPP_INFO(this->get_logger(), "Set Running State and Control Cost");

        // initial state constraint
        SX initial_state_constraint = reshape(X(Slice(), 0) - initial_state, -1, 1);
        RCLCPP_INFO(this->get_logger(), "Initial state constraint size1: %1lld", initial_state_constraint.size1());
        RCLCPP_INFO(this->get_logger(), "Initial state constraint size2: %1lld", initial_state_constraint.size2());
        RCLCPP_INFO(this->get_logger(), "Set Initial State Constraint");

        // final state constraint
        SX final_state_constraint = reshape(X(Slice(0, 2), N) - final_position, -1, 1);
        RCLCPP_INFO(this->get_logger(), "Final state constraint size1: %1lld", final_state_constraint.size1());
        RCLCPP_INFO(this->get_logger(), "Final state constraint size2: %1lld", final_state_constraint.size2());
        RCLCPP_INFO(this->get_logger(), "Set Final State Constraint");

        // initial control constraint
        SX initial_control_constraint = reshape(U(Slice(), 0), -1, 1);
        RCLCPP_INFO(this->get_logger(), "Initial Control Constraint size1: %1lld", initial_control_constraint.size1());
        RCLCPP_INFO(this->get_logger(), "Initial Control Constraint size2: %1lld", initial_control_constraint.size2());
        RCLCPP_INFO(this->get_logger(), "Set Initial Control Constraint");

        // final control constraint
        SX final_control_constraint = reshape(U(Slice(), N-1), -1, 1);
        RCLCPP_INFO(this->get_logger(), "Final Control Constraint size1: %1lld", final_control_constraint.size1());
        RCLCPP_INFO(this->get_logger(), "Final Control Constraint size2: %1lld", final_control_constraint.size2());
        RCLCPP_INFO(this->get_logger(), "Set Final Control Constraint");

        // add acceleration constraint
        SX v_dot_constraint = reshape((1/dt)*(U(0, Slice(1, N)) - U(0, Slice(0, N-1))), -1, 1);
        SX r_dot_constraint = reshape((1/dt)*(U(1, Slice(1, N)) - U(1, Slice(0, N-1))), -1, 1);
        RCLCPP_INFO(this->get_logger(), "V_dot Constraint size1: %1lld", v_dot_constraint.size1());
        RCLCPP_INFO(this->get_logger(), "V_dot Constraint size2: %1lld", v_dot_constraint.size2());
        RCLCPP_INFO(this->get_logger(), "Set V_dot Constraint");
        RCLCPP_INFO(this->get_logger(), "R_dot Constraint size1: %1lld", r_dot_constraint.size1());
        RCLCPP_INFO(this->get_logger(), "R_dot Constraint size2: %1lld", r_dot_constraint.size2());
        RCLCPP_INFO(this->get_logger(), "Set R_dot Constraint");

        // dynamics constraints
        SX x_now = X(Slice(), Slice(0, N));
        SX delta_x = dt * vertcat(
            U(0, Slice()) * cos(X(2, Slice(0, N))),
            U(0, Slice()) * sin(X(2, Slice(0, N))),
            U(1, Slice()));
        SX x_next = x_now + delta_x;
        RCLCPP_INFO(this->get_logger(), "x_now size1: %1lld", x_now.size1());
        RCLCPP_INFO(this->get_logger(), "x_now size2: %1lld", x_now.size2());
        RCLCPP_INFO(this->get_logger(), "delta_x size1: %1lld", delta_x.size1());
        RCLCPP_INFO(this->get_logger(), "delta_x size2: %1lld", delta_x.size2());

        SX dynamics_constraint = reshape(x_next - X(Slice(), Slice(1, N+1)), -1, 1);
        RCLCPP_INFO(this->get_logger(), "Dynamnics constraint size1: %1lld", dynamics_constraint.size1()); 
        RCLCPP_INFO(this->get_logger(), "Dynamnics constraint size2: %1lld", dynamics_constraint.size2()); 
        RCLCPP_INFO(this->get_logger(), "x_next: %s", x_next.get_str().c_str());
        RCLCPP_INFO(this->get_logger(), "Set Dynamics Constraint");

        // polyhedron constraints
        vector<SX> polyhedron_constraint_vector;
        int last_k = 0;
        for(size_t i=0; i < polyhedrons.size(); ++i){
            int next_k = static_cast<int>(path_proportions[i] * N);
            RCLCPP_INFO(this->get_logger(), "path_proportions[i]: %1f", path_proportions[i]);
            RCLCPP_INFO(this->get_logger(), "last_k: %1d", last_k);
            RCLCPP_INFO(this->get_logger(), "next_k: %1d", next_k);
            vec_E<Hyperplane2D> hyperplanes = polyhedrons[i].hyperplanes();
            for(int k=last_k; k < next_k; ++k){
                // ensure point is in the polyhedron
                for(Hyperplane2D hyperplane : hyperplanes){
                    Vec2f normal = hyperplane.n_;
                    Vec2f point = hyperplane.p_;
                    SX value = normal[0]*(X(0, k) - point[0]) + normal[1]*(X(1, k) - point[1]);
                    polyhedron_constraint_vector.push_back(value);
                }
            }
            last_k = next_k;
        }
        SX polyhedron_constraint = vertcat(polyhedron_constraint_vector);
        RCLCPP_INFO(this->get_logger(), "Polyhedron constraint size1: %1lld", polyhedron_constraint.size1()); 
        RCLCPP_INFO(this->get_logger(), "Polyhedron constraint size2: %1lld", polyhedron_constraint.size2()); 
        RCLCPP_INFO(this->get_logger(), "Set Polyhedron Constraint");
        
        SX equality_constraints = vertcat(
            initial_state_constraint, 
            final_state_constraint,
            initial_control_constraint,
            final_control_constraint,
            dynamics_constraint,
            v_dot_constraint);
        SX constraints = vertcat(equality_constraints, r_dot_constraint, polyhedron_constraint);

        // set up NLP solver and solve the program
        Function solver = nlpsol("solver", "ipopt", SXDict{
            {"x", variables_flat},
            {"f", objective},
            {"g", constraints}
        });

        // Set constraint bounds
        DM zero_bg_constraints = vertcat(
            DM::zeros(initial_state_constraint.size1(), 1), 
            DM::zeros(final_state_constraint.size1(), 1),
            DM::zeros(initial_control_constraint.size1(), 1),
            DM::zeros(final_control_constraint.size1(), 1),
            DM::zeros(dynamics_constraint.size1(), 1));

        DM lbg = vertcat(
            zero_bg_constraints, 
            -DM::ones(v_dot_constraint.size1(), 1),
            -(pi/4)*DM::ones(r_dot_constraint.size1(), 1),
            -DM::inf(polyhedron_constraint.size1(), 1));
        DM ubg = vertcat(
            zero_bg_constraints,
            DM::ones(v_dot_constraint.size1(), 1),
            (pi/4)*DM::ones(r_dot_constraint.size1(), 1),
            DM::zeros(polyhedron_constraint.size1(), 1));

        RCLCPP_INFO(this->get_logger(), "Constriants (G) Lower Bound size1: %1lld", lbg.size1()); 
        RCLCPP_INFO(this->get_logger(), "Constriants (G) Lower Bound size2: %1lld", lbg.size2()); 
        RCLCPP_INFO(this->get_logger(), "Constriants (G) Upper Bound size1: %1lld", ubg.size1()); 
        RCLCPP_INFO(this->get_logger(), "Constriants (G) Upper Bound size2: %1lld", ubg.size2()); 

        // Flatten decision variable bounds
        DM lbx = pack_variables_fn(lower_bounds)[0];
        DM ubx = pack_variables_fn(upper_bounds)[0];
        RCLCPP_INFO(this->get_logger(), "Decision Variables Lower Bound size1: %1lld", lbx.size1()); 
        RCLCPP_INFO(this->get_logger(), "Decision Variables Lower Bound size2: %1lld", lbx.size2()); 
        RCLCPP_INFO(this->get_logger(), "Decision Variables Upper Bound size1: %1lld", ubx.size1()); 
        RCLCPP_INFO(this->get_logger(), "Decision Variables Upper Bound size2: %1lld", ubx.size2()); 
        
        // Initial guess for optimization
        DM initial_guess = DM::zeros(variables_flat.size1(), 1);
        for (int i = 0; i < N + 1; ++i) {
            double alpha = static_cast<double>(i) / N;
            initial_guess(3 * i) = (1 - alpha) * initial_state(0) + alpha * final_position(0);
            initial_guess(3 * i + 1) = (1 - alpha) * initial_state(1) + alpha * final_position(1);
            // initial_guess(3 * i + 2) = (1 - alpha) * initial_state(2) + alpha * final_state(2);
        }
        RCLCPP_INFO(this->get_logger(), "Initial Guess size1: %1lld", initial_guess.size1()); 
        RCLCPP_INFO(this->get_logger(), "Initial Guess size2: %1lld", initial_guess.size2()); 

        // Solve NLP
        map<string, DM> solver_args = {
            {"x0", initial_guess},   
            {"lbx", lbx}, 
            {"ubx", ubx}, 
            {"lbg", lbg},
            {"ubg", ubg}
            };

        RCLCPP_INFO(this->get_logger(), "Solving NLP");
        map<string, DM> solver_result = solver(solver_args);
        RCLCPP_INFO(this->get_logger(), "Optimization complete");

        DM solution = solver_result["x"];
        DMVector unpacked_solution = unpack_variables_fn(solution);

        // Store solution in a trajectory container
        vector<vector<double>> trajectory;
        for (int i = 0; i < N+1; ++i) {
            vector<double> state{
                static_cast<double>(unpacked_solution[0](0, i)), 
                static_cast<double>(unpacked_solution[0](1, i)), 
                static_cast<double>(unpacked_solution[0](2, i))};
            trajectory.push_back(state);
        }
        RCLCPP_INFO(this->get_logger(), "Generated trajectory with %ld points", trajectory.size());
        publishTrajectory(trajectory);
        RCLCPP_INFO(this->get_logger(), "Published Trajectory");

        // log the v, omega, x, y, and theta values for plotting

        // allow to read in bounds from a config file

        // RCLCPP_INFO(this->get_logger(), "X Optimal: %s", unpacked_solution[0].get_str().c_str());
        // RCLCPP_INFO(this->get_logger(), "U Optimal: %s", unpacked_solution[1].get_str().c_str());
    }

    double yaw_from_quaternion(geometry_msgs::msg::Quaternion& quaternion){
        return atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 
        1.0 - 2.0 * (pow(quaternion.y, 2) + pow(quaternion.z, 2)));
    }

    void publishTrajectory(vector<vector<double>>& trajectory){
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        visualization_msgs::msg::Marker path_points;
        path_points.header.stamp = this->now();
        path_points.header.frame_id = "map";
        path_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        path_points.action = visualization_msgs::msg::Marker::ADD;
        path_points.scale.x = 0.1;
        path_points.scale.y = 0.1;
        path_points.scale.z = 0.1;
        path_points.color.r = 1.0; 
        path_points.color.g = 0.0;
        path_points.color.b = 0.0;
        path_points.color.a = 1.0;

        for (const vector<double>& state : trajectory) {
            // add a pose to the path message
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";

            pose.pose.position.x = state[0];
            pose.pose.position.y = state[1];
            pose.pose.position.z = 0.0;
            path_msg.poses.push_back(pose);

            // add a point to the marker message
            geometry_msgs::msg::Point point;
            point.x = state[0];
            point.y = state[1];
            point.z = 0.1;
            path_points.points.push_back(point);
        }

        mpc_path_pub_->publish(path_msg);
        mpc_path_points_->publish(path_points);
        RCLCPP_INFO(this->get_logger(), "Published trajectory with %ld points", trajectory.size());
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<custom_msgs_pkg::msg::PolygonArray>::SharedPtr polygon_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr risk_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr travel_corridors_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mpc_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mpc_path_points_;

    nav_msgs::msg::Odometry::SharedPtr odometry_;
    vec_Vec2f obstacles_;
    nav_msgs::msg::OccupancyGrid::SharedPtr risk_map_;
    DM global_to_lower_left_;
    DM lower_left_to_risk_center_;
    DM global_risk_to_center_;
    Function continuous_risk_map_;
    

    // casadi optimization relevant declarations
    Function pack_variables_fn_;
    Function unpack_variables_fn_;
    DMVector lower_bounds_;
    DMVector upper_bounds_;
    SX cost_;
    DM x_opt_;
    DM u_opt_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    shared_ptr<MPCPlannerCorridors> node = make_shared<MPCPlannerCorridors>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
