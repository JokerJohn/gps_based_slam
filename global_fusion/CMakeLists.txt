cmake_minimum_required(VERSION 2.8.3)
project(global_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(Ceres REQUIRED)

add_subdirectory(./ThirdParty/GeographicLib/)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ./ThirdParty/GeographicLib/include/
)


catkin_package()


add_executable(global_fusion_node 
	src/globalOptNode.cpp
	src/globalOpt.cpp)

target_link_libraries(global_fusion_node ${catkin_LIBRARIES} ${CERES_LIBRARIES} libGeographiccc)

# 利用gps和imu做一个简单的里程计，主要是坐标对齐
add_executable(gps_node src/gps.cpp src/gpsTools.cpp)
target_link_libraries(gps_node ${catkin_LIBRARIES} )