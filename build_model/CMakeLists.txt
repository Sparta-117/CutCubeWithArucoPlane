cmake_minimum_required(VERSION 2.8.3)
project(build_model)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")

############### dependencies ######################
#Opencv
find_package(OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIR})
#PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
# Eigen
include_directories( "/usr/include/eigen3" )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
#ros
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
)
include_directories(${catkin_INCLUDE_DIRS})

#self file
include_directories("~/new_sr300_build_model/devel/include")
include_directories(   include/build_model)
include_directories(	include) 

############### e ######################
set(SRC src/camera.cpp src/config.cpp src/qrplane.cpp) 

add_executable(build_model src/main.cpp ${SRC}) 
add_executable(get_image_node src/get_image.cpp)

target_link_libraries(build_model
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
   ${Sophus_LIBRARIES}
 )
target_link_libraries(get_image_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${PCL_LIBRARIES}
 )



