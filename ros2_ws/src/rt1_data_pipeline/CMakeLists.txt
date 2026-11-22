cmake_minimum_required(VERSION 3.8)
project(rt1_data_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rt1_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# --- Pure library (no ROS): probe metrics ---
add_library(probe_metrics src/probe_metrics.cpp)
target_include_directories(probe_metrics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(probe_metrics ${OpenCV_LIBS})

# --- Node: dual camera stitcher ---
add_executable(dual_cam_stitcher_node src/dual_cam_stitcher_node.cpp)
target_include_directories(dual_cam_stitcher_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(dual_cam_stitcher_node rclcpp sensor_msgs cv_bridge message_filters)
target_link_libraries(dual_cam_stitcher_node ${OpenCV_LIBS})

# --- Node: probe metrics ---
add_executable(probe_metrics_node src/probe_metrics_node.cpp)
ament_target_dependencies(probe_metrics_node rclcpp sensor_msgs std_srvs cv_bridge rt1_interfaces)
target_link_libraries(probe_metrics_node probe_metrics)

install(TARGETS dual_cam_stitcher_node probe_metrics_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()