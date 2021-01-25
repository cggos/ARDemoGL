cmake_minimum_required(VERSION 2.8.3)
project(ar_demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
add_definitions(-w)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions  
  image_transport
  sensor_msgs
  cv_bridge
  message_filters
)

catkin_package()

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
if(Pangolin_FOUND)
   include_directories(${Pangolin_INCLUDE_DIRS})
endif()
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

include(3rdparties/camera_models/camera_models.cmake)

add_executable(ar_demo_node src/ar_demo_node.cpp src/ViewerAR.cpp ${camera_models_src})

target_link_libraries(ar_demo_node ${catkin_LIBRARIES} ${camera_models_libs} ${Pangolin_LIBRARIES})


