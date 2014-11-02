# %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(ff_cameras)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs camera_info_manager image_transport)

## Declare ROS messages and services

#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs )

## Declare a catkin package
catkin_package( INCLUDE_DIRS include
#Libraries
CATKIN_DEPENDS  sensor_msgs geometry_msgs camera_info_manager image_transport)
include_directories("/usr/include/flycapture")
find_library(FLYCAPTURE2 flycapture)
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(pgrff_node src/pgrff_node.cpp)
target_link_libraries(pgrff_node ${catkin_LIBRARIES} ${FLYCAPTURE2}  ${Boost_LIBRARIES})
add_dependencies(pgrff_node std_msgs sensor_msgs camera_info_manager image_transport)
add_executable(ff_node src/ff_node.cpp)
target_link_libraries(ff_node ${catkin_LIBRARIES} ${FLYCAPTURE2}  ${Boost_LIBRARIES})
add_dependencies(ff_node std_msgs sensor_msgs camera_info_manager image_transport)


#%EndTag(FULLTEXT)%

