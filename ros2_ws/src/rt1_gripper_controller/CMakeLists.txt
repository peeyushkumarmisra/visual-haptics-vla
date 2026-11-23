cmake_minimum_required(VERSION 3.8)
project(rt1_gripper_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_library(rt1_gripper_controller SHARED src/gripper_centering_controller.cpp)
target_compile_features(rt1_gripper_controller PUBLIC cxx_std_17)
ament_target_dependencies(rt1_gripper_controller
    controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs)

# Lets controller_manager find the class by name at run time.
pluginlib_export_plugin_description_file(controller_interface gripper_centering_controller.xml)

install(TARGETS rt1_gripper_controller
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

ament_export_libraries(rt1_gripper_controller)
ament_export_dependencies(controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs)
ament_package()