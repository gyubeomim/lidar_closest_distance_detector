cmake_minimum_required(VERSION 2.8.3)
project(lidar_closest_distance_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  jsk_recognition_msgs
  cv_bridge
  )

find_package(OpenCV REQUIRED)

FIND_PACKAGE(OpenMP)
IF(OPENMP_FOUND)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Found OpenMP")
ENDIF()

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_executable(lcdd_convex_polygon_nodelet
  src/convex_polygon.cpp
  src/convex_polygon_node.cpp
  )
add_dependencies(lcdd_convex_polygon_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lcdd_convex_polygon_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# nodelets
add_library(lcdd_registration_nodelet
  src/registration.cpp
  )
add_dependencies(lcdd_registration_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lcdd_registration_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

add_library(lcdd_segmentation_nodelet
  src/segmentation.cpp
  )
add_dependencies(lcdd_segmentation_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lcdd_segmentation_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

add_library(lcdd_filtering_nodelet
  src/filtering.cpp
  )
add_dependencies(lcdd_filtering_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lcdd_filtering_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

add_library(lcdd_distance_detector_nodelet
  src/distance_detector.cpp
  )
add_dependencies(lcdd_distance_detector_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lcdd_distance_detector_nodelet
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
