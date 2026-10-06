cmake_minimum_required(VERSION 3.8)
project(pronto_quadruped_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wall)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pronto_utils)
find_package(pronto_core REQUIRED)
find_package(pronto_ros REQUIRED)
find_package(pronto_quadruped REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pronto_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdf REQUIRED)
find_package(pi3hat_moteus_int_msgs REQUIRED)



add_library(${PROJECT_NAME} SHARED  src/legodo_handler_ros.cpp
                                    src/stance_estimator_ros.cpp
                                    src/force_sensor_stance_estimator_ros.cpp
                                    src/foot_sensor_stance_detector_ros.cpp
                                    src/conversions.cpp
                                    src/leg_odometer_ros.cpp
                                    src/pronto_mammal_utilities.cpp
                                    src/dynamic_stance_estimator_ros.cpp
                                    src/bias_lock_handler_ros.cpp
                                    src/quad_model_parser.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pronto_quadruped_ros
  "rclcpp"
  "pronto_utils"
  "pronto_core"
  "pronto_ros"
  "pronto_quadruped"
  "geometry_msgs"
  "sensor_msgs"
  "pronto_msgs"
  "Eigen3"
  "visualization_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "urdf"
  "pi3hat_moteus_int_msgs"
)
# pinocchio >= 3 exports only CMake targets, which ament_target_dependencies does not pick up
target_link_libraries(pronto_quadruped_ros pinocchio::pinocchio)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pronto_quadruped_ros PRIVATE "PRONTO_QUADRUPED_ROS_BUILDING_LIBRARY")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies("rclcpp"
"pronto_utils"
"pronto_core"
"pronto_ros"
"pronto_quadruped"
"geometry_msgs"
"sensor_msgs"
"pronto_msgs"
"Eigen3"
"visualization_msgs"
"tf2_ros"
"tf2_geometry_msgs"
"nav_msgs"
"pinocchio"
  "urdf"
  "pi3hat_moteus_int_msgs")


install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS pronto_quadruped_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

ament_export_include_directories(
  include
)
ament_export_libraries(
  pronto_quadruped_ros
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
