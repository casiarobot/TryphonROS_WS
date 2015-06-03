# %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(sensors)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs geometry_msgs genmsg)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES sonar.msg sonarArray.msg compass.msg motor.msg motorArray.msg forces.msg imuros.msg imubuff.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

## Declare a catkin package
catkin_package( INCLUDE_DIRS include
#Libraries
CATKIN_DEPENDS  sensor_msgs geometry_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(thruster src/thruster.cpp src/robocomm.cpp)
target_link_libraries(thruster ${catkin_LIBRARIES})
add_dependencies(thruster sensors_gencpp std_msgs geometry_msgs)

add_executable(sensors_thrusters src/sensors_thrusters.cpp src/robocomm.cpp)
target_link_libraries(sensors_thrusters ${catkin_LIBRARIES})
add_dependencies(sensors_thrusters sensors_gencpp std_msgs geometry_msgs)

#%EndTag(FULLTEXT)%

