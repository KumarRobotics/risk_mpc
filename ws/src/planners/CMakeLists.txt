cmake_minimum_required(VERSION 3.8)
project(planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(custom_msgs_pkg REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(decomp_ros_util REQUIRED)
find_package(decomp_util REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(casadi REQUIRED)

add_executable(mpc_planner_corridors src/mpc_planner_corridors.cpp)
target_link_libraries(mpc_planner_corridors casadi)
target_include_directories(mpc_planner_corridors PRIVATE include ${DECOMP_UTIL_INCLUDE_DIRS} ${DECOMP_ROS_UTIL_INCLUDE_DIRS})
ament_target_dependencies(mpc_planner_corridors
  rclcpp 
  sensor_msgs 
  nav_msgs 
  geometry_msgs
  visualization_msgs
  custom_msgs_pkg
  decomp_ros_util
  decomp_util 
  decomp_ros_msgs
  pcl_conversions 
  casadi
)

install(TARGETS
mpc_planner_corridors
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/jump_point_planner.py
  scripts/occupancy_grid_parser.py
  scripts/mpc_planner_max_approx_cost_risk.py
  scripts/mpc_planner_max_approx_cost.py
  scripts/a_star_planner.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
