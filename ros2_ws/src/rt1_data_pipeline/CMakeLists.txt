cmake_minimum_required(VERSION 3.8)
project(rt1_data_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rt1_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# --- Pure library (no ROS): probe metrics ---
add_library(probe_metrics src/probe_metrics.cpp)
target_include_directories(probe_metrics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(probe_metrics ${OpenCV_LIBS})

# --- 2.1 Stitcher ---
add_executable(dual_cam_stitcher_node src/dual_cam_stitcher_node.cpp)
target_include_directories(dual_cam_stitcher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(dual_cam_stitcher_node rclcpp sensor_msgs cv_bridge message_filters)
target_link_libraries(dual_cam_stitcher_node ${OpenCV_LIBS})

# --- 2.2 Probe metrics ---
add_executable(probe_metrics_node src/probe_metrics_node.cpp)
ament_target_dependencies(probe_metrics_node rclcpp sensor_msgs std_srvs cv_bridge rt1_interfaces)
target_link_libraries(probe_metrics_node probe_metrics)

# --- 2.4 Expert FSM ---
add_executable(probe_controller_node src/probe_controller_node.cpp)
ament_target_dependencies(probe_controller_node
  rclcpp sensor_msgs std_msgs std_srvs trajectory_msgs rt1_interfaces)

install(TARGETS dual_cam_stitcher_node probe_metrics_node probe_controller_node
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/mystery_spawner.py
  scripts/episode_logger.py
  scripts/collect_episodes.py
  scripts/audit_dataset.py
  scripts/replay_episode.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()