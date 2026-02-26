cmake_minimum_required(VERSION 3.20)
project(nav2_coverage_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_coverage_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

nav2_package()

add_library(nav2_coverage_core INTERFACE)
target_include_directories(nav2_coverage_core
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(nav2_coverage_core INTERFACE
  ${geometry_msgs_TARGETS}
  ${nav2_coverage_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_action::rclcpp_action
  tf2_ros::tf2_ros
)

install(TARGETS nav2_coverage_core
  EXPORT nav2_coverage_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs
  rclcpp_lifecycle
  tf2_ros
  rclcpp_action
  nav_msgs
  nav2_coverage_msgs
)
ament_export_targets(nav2_coverage_core)

ament_package()
