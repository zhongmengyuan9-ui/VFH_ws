cmake_minimum_required(VERSION 3.8)
project(mower_vfh)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add executable
add_executable(depth_vfh_controller src/depth_vfh_controller.cpp)

# Include directories
target_include_directories(depth_vfh_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(depth_vfh_controller
  Eigen3::Eigen
)

# Link ROS 2 packages
ament_target_dependencies(depth_vfh_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

# Install targets
install(TARGETS depth_vfh_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files (if exists)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()