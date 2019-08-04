cmake_minimum_required(VERSION 2.8.3)
project(lcdd_loam)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(lcdd_scanRegistration src/lcdd_scanRegistration.cpp)
add_executable(lcdd_laserOdometry src/lcdd_laserOdometry.cpp)
add_executable(lcdd_laserMapping src/lcdd_laserMapping.cpp)
add_executable(lcdd_transformMaintenance src/lcdd_transformMaintenance.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lcdd_scanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(lcdd_laserOdometry ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(lcdd_laserMapping ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(lcdd_transformMaintenance ${catkin_LIBRARIES} ${PCL_LIBRARIES})
