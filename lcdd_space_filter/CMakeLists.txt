cmake_minimum_required(VERSION 2.8.3)
project(lcdd_space_filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        velodyne_pointcloud
        tf
        )

catkin_package(CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        cv_bridge
        velodyne_pointcloud
        tf
        )

find_package(Qt5Core REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenMP)

include_directories(
        ${catkin_INCLUDE_DIRS}
        # include
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})

add_executable(lcdd_space_filter_node src/space_filter.cpp)
target_link_libraries(lcdd_space_filter_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
add_dependencies(lcdd_space_filter_node ${catkin_EXPORTED_TARGETS})
