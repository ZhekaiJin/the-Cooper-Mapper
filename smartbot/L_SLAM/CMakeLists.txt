cmake_minimum_required(VERSION 2.8.3)
project(lidar_slam)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

# work around for proj4
set({PROJ4_LIBRARIES proj)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  nodelet
  map_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  set(OpenMP_LIBS gomp)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)
#find_package(PROJ4 REQUIRED)
#include_directories(${PROJ4_INCLUDE_DIR})

find_package(yaml-cpp REQUIRED)
link_libraries(${YAML_CPP_LIBRARIES})

INCLUDE(cmake_modules/FindG2O.cmake)
#find_package(G2O REQUIRED)
include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})

include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  src)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs hdmap_msgs
  DEPENDS EIGEN3 PCL OpenCV
  LIBRARIES loam
)

add_definitions( -march=native )

add_subdirectory(src/frame_match)
add_subdirectory(src/odometry)
add_subdirectory(src/pose_graph)
add_subdirectory(src/io)
add_subdirectory(src/kf_fusion)
add_subdirectory(src/map_evaluation)
#add_subdirectory(src/tests)

