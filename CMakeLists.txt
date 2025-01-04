cmake_minimum_required(VERSION 3.8)
project(ffastllamaa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(external_lib)

# Add C++ 17 support
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mavx -mbmi2")

# Set release mode
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Ceres 2.2 REQUIRED)
find_package(message_filters REQUIRED)


find_package(rosidl_default_generators REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryDistField.srv"
  DEPENDENCIES std_msgs
)


### Specify additional locations of header files
### Your package locations should be listed before other locations
#include_directories(
#  include
#  ${EIGEN_INCLUDE_DIR}
#  ${CERES_INCLUDE_DIRS}
#  external_lib
#)


add_executable(scan_maker src/scan_maker_node.cpp)
add_executable(lidar_feature_detection src/lidar_feature_detection_node.cpp)
add_executable(lidar_scan_odometry src/lidar_odometry_node.cpp src/lib/lidar_odometry.cpp src/lib/state.cpp)
add_executable(gp_map src/gp_map_node.cpp src/lib/map_distance_field.cpp)

add_executable(field_visualiser src/field_visualiser_node.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(scan_maker
  Eigen3::Eigen
)
target_link_libraries(lidar_feature_detection
  Eigen3::Eigen
)
target_link_libraries(lidar_scan_odometry
  Eigen3::Eigen
  Ceres::ceres
)
target_link_libraries(gp_map
  Eigen3::Eigen
  phtree
  Ceres::ceres
  "${cpp_typesupport_target}"
  PoissonRecon
)
target_link_libraries(field_visualiser
  Eigen3::Eigen
  "${cpp_typesupport_target}"
)


#ament_export_dependencies(rosidl_default_runtime)
#rosidl_target_interfaces(gp_map ${PROJECT_NAME} "rosidl_typesupport_cpp")
#rosidl_target_interfaces(field_visualiser ${PROJECT_NAME} "rosidl_typesupport_cpp")



ament_target_dependencies(scan_maker
  rclcpp
  sensor_msgs
  Eigen3)

ament_target_dependencies(lidar_feature_detection
  rclcpp
  sensor_msgs
  Eigen3)

ament_target_dependencies(lidar_scan_odometry
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_filters
  std_msgs
  tf2
  tf2_ros
  Eigen3
  Ceres)
  
ament_target_dependencies(gp_map
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  std_msgs
  tf2
  tf2_ros
  Eigen3
  Ceres)

ament_target_dependencies(field_visualiser
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
  Eigen3)


include_directories(
  include
  external_lib

)

# Enable openMP
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

install(
    DIRECTORY include/ external_lib/
    DESTINATION include
    )
install(TARGETS
  scan_maker lidar_feature_detection lidar_scan_odometry gp_map field_visualiser
  DESTINATION lib/${PROJECT_NAME})





install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)


ament_package()
