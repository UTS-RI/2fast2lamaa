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


find_package(Eigen3 REQUIRED)
find_package(Ceres 2.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_filters
  std_msgs
  tf2
  tf2_ros
  message_generation
)

add_service_files(
  FILES
  QueryDistField.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime roscpp sensor_msgs geometry_msgs std_msgs tf2 tf2_ros message_filters)

add_executable(scan_maker src/scan_maker_node.cpp)
add_executable(lidar_feature_detection src/lidar_feature_detection_node.cpp)
add_executable(lidar_scan_odometry src/lidar_odometry_node.cpp src/lib/lidar_odometry.cpp src/lib/state.cpp)
add_executable(gp_map src/gp_map_node.cpp src/lib/map_distance_field.cpp)

add_executable(field_visualiser src/field_visualiser_node.cpp)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  external_lib

)

target_link_libraries(scan_maker
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(lidar_feature_detection
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

target_link_libraries(lidar_scan_odometry
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  Ceres::ceres
)

target_link_libraries(gp_map
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  phtree
  Ceres::ceres
  PoissonRecon
)


target_link_libraries(field_visualiser
  ${catkin_LIBRARIES}
  Eigen3::Eigen
)

add_dependencies(gp_map ffastllamaa_gencpp)
add_dependencies(field_visualiser ffastllamaa_gencpp)

# Enable openMP
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
