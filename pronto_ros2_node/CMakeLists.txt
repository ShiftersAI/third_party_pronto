cmake_minimum_required(VERSION 3.8)
project(pronto_ros2_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # upstream code is not clean for these; keep real problems (e.g. -Wreturn-type, -Wformat) visible
  add_compile_options(-Wno-unused-parameter -Wno-reorder -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pronto_ros REQUIRED)
find_package(pronto_quadruped_ros REQUIRED)
find_package(pronto_quadruped_commons REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pi3hat_moteus_int_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(
  external_sensor SHARED
  src/qualysis_mt.cpp
  src/wheeled_odometry.cpp
)



ament_target_dependencies(
  external_sensor

  rclcpp
  pronto_core
  gazebo_msgs
  sensor_msgs
  nav_msgs
  std_srvs
  mocap4r2_msgs
)



target_include_directories(external_sensor PUBLIC

$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

$<INSTALL_INTERFACE:include>)

install(
  TARGETS
  external_sensor
  EXPORT export_external_sensor
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
add_executable(pronto_node src/pronto_ros2_node.cpp)

ament_target_dependencies(
  pronto_node
  rclcpp
  pronto_ros
  pronto_quadruped_ros
  pronto_quadruped_commons
  sensor_msgs
  nav_msgs
  pi3hat_moteus_int_msgs
  # mocap_msgs
  gazebo_msgs
  std_srvs
)
# pinocchio >= 3 exports only CMake targets, which ament_target_dependencies does not pick up
target_link_libraries(
  pronto_node  external_sensor pinocchio::pinocchio
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

install(TARGETS
pronto_node
  DESTINATION lib/${PROJECT_NAME})


install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  external_sensor
)

ament_package()
