 # %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(cameras_tryphon)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs camera_info_manager image_transport dynamic_reconfigure)

## Declare ROS messages and services

#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs )

generate_dynamic_reconfigure_options(
  cfg/cameras.cfg
  cfg/cameras_freerun.cfg
)

## Declare a catkin package
catkin_package( #INCLUDE_DIRS include
#Libraries
CATKIN_DEPENDS  sensor_msgs geometry_msgs camera_info_manager image_transport)
include_directories("/usr/include/flycapture")
find_library(FLYCAPTURE2 flycapture)
## Build talker and listener
#include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(pgrff_tryphon_software src/pgrff_tryphon_software.cpp)
target_link_libraries(pgrff_tryphon_software ${catkin_LIBRARIES} ${FLYCAPTURE2}  ${Boost_LIBRARIES})
add_dependencies(pgrff_tryphon_software std_msgs sensor_msgs camera_info_manager image_transport ${PROJECT_NAME}_gencfg)

add_executable(pgrff_tryphon_software2 src/pgrff_tryphon_software2.cpp)
target_link_libraries(pgrff_tryphon_software2 ${catkin_LIBRARIES} ${FLYCAPTURE2}  ${Boost_LIBRARIES})
add_dependencies(pgrff_tryphon_software2 std_msgs sensor_msgs camera_info_manager image_transport ${PROJECT_NAME}_gencfg)

#add_executable(pgrff_node_freerun src/pgrff_node_freerun.cpp)
#target_link_libraries(pgrff_node_freerun ${catkin_LIBRARIES} ${FLYCAPTURE2}  ${Boost_LIBRARIES})
#add_dependencies(pgrff_node_freerun std_msgs sensor_msgs camera_info_manager image_transport)

#add_executable(pgrff_node_exttrig src/pgrff_node_exttrig.cpp)
#target_link_libraries(pgrff_node_exttrig ${catkin_LIBRARIES} ${FLYCAPTURE2}  ${Boost_LIBRARIES})
#add_dependencies(pgrff_node_exttrig std_msgs sensor_msgs camera_info_manager image_transport)


#%EndTag(FULLTEXT)%

