cmake_minimum_required(VERSION 2.8.3)
project(links)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs genmsg message_generation)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES imubuff.msg imuros.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(
DEPENDENCIES 
std_msgs geometry_msgs sensor_msgs)

## Declare a catkin package
catkin_package( INCLUDE_DIRS 
#Libraries
CATKIN_DEPENDS  sensor_msgs geometry_msgs message_runtime)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(links_imubuff src/links_imubuff.cpp)
target_link_libraries(links_imubuff  ${catkin_LIBRARIES})
add_dependencies(links_imubuff  sensor_msgs ${PROJECT_NAME}_gencpp)

add_executable(link_imubuff src/link_imubuff.cpp)
target_link_libraries(link_imubuff  ${catkin_LIBRARIES})
add_dependencies(link_imubuff  sensor_msgs ${PROJECT_NAME}_gencpp)


#%EndTag(FULLTEXT)%


