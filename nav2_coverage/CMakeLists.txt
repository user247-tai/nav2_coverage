cmake_minimum_required(VERSION 3.20)
project(nav2_coverage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_coverage_core REQUIRED)
find_package(nav2_coverage_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)

nav2_package()

set(executable_name coverage_server)
set(library_name ${executable_name}_core)

add_library(${library_name} SHARED
  src/coverage_server.cpp
)

target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${library_name} PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_coverage_core::nav2_coverage_core
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_coverage_msgs_TARGETS}
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2_ros::tf2_ros
)

target_link_libraries(${library_name} PRIVATE
  ${lifecycle_msgs_TARGETS}
  rclcpp_components::component
)

add_executable(${executable_name}
  src/main.cpp
)

target_include_directories(${executable_name}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${executable_name} PRIVATE
  rclcpp::rclcpp
  ${library_name}
)

rclcpp_components_register_nodes(${library_name} "nav2_coverage::CoverageServer")

add_library(fullmap_pose_creator SHARED plugins/fullmap_pose_creator.cpp)
add_library(zones_pose_creator SHARED plugins/zones_pose_creator.cpp)
target_include_directories(fullmap_pose_creator
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(fullmap_pose_creator PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_coverage_core::nav2_coverage_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
)
target_link_libraries(fullmap_pose_creator PRIVATE
  nav2_util::nav2_util_core
  pluginlib::pluginlib
)

target_include_directories(zones_pose_creator
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(zones_pose_creator PUBLIC
  ${geometry_msgs_TARGETS}
  nav2_coverage_core::nav2_coverage_core
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${rcl_interfaces_TARGETS}
  tf2_ros::tf2_ros
)
target_link_libraries(zones_pose_creator PRIVATE
  nav2_util::nav2_util_core
  pluginlib::pluginlib
)

pluginlib_export_plugin_description_file(nav2_coverage_core plugins.xml)

install(TARGETS fullmap_pose_creator zones_pose_creator ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})

ament_export_libraries(${library_name})

ament_export_dependencies(
  geometry_msgs
  nav_msgs
  nav2_coverage_msgs
  nav2_coverage_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rcl_interfaces
  tf2_ros
)

ament_package()
