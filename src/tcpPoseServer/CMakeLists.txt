cmake_minimum_required(VERSION 2.8.3)
project(tcp_pose_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  sensor_msgs
  state
#  cmake_modules
)
#find_package(OpenCV REQUIRED)
FIND_PACKAGE (Threads)
find_package(Eigen REQUIRED)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS message_runtime roscpp cv_bridge sensor_msgs state
  DEPENDS eigen OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}  
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp executable
# add_executable(beginner_tutorials_node src/beginner_tutorials_node.cpp)
#add_executable(beginner_tutorials src/send_image.cpp)
add_executable( tcp_pose_server
    src/main.cpp
    src/Server.cpp
)
add_dependencies(sensors_gencpp std_msgs geometry_msgs sensors state)
include_directories( ${ZLIB_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
FIND_PACKAGE (ZLIB REQUIRED)
target_link_libraries(tcp_pose_server
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   pthread
)
