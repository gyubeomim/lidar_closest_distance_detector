cmake_minimum_required(VERSION 2.8.3)
project(lcdd_ring_noise_remover)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  visualization_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
# include
${catkin_INCLUDE_DIRS}
${OpenCV_INCLUDE_DIRS}
)

add_executable(lcdd_ring_noise_remover_node src/ring_noise_remover_node.cpp)

target_link_libraries(lcdd_ring_noise_remover_node
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBRARIES}
                     )
