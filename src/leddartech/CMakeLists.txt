# %Tag(FULLTEXT)%


cmake_minimum_required(VERSION 2.8.3)
project(leddartech)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES leddar_data.msg )
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package( INCLUDE_DIRS include
#Libraries
CATKIN_DEPENDS   geometry_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

#add_executable(leddartech src/leddartech.cpp src/robocomm.cpp)
add_executable(leddar_one src/leddar_one.cpp
                      #  src/libleddar/Definitions.h
                       # src/libleddar/UserDefs.h
                       # src/libleddar/OS.h
                       # src/libleddar/Modbus.h
                       # src/libleddar/Leddar.h
                        src/libleddar/OS.cpp
                        src/libleddar/Modbus.cpp
                        src/libleddar/Leddar.cpp)
target_link_libraries(leddar_one ${catkin_LIBRARIES})
add_dependencies(leddar_one leddartech_gencpp)


#%EndTag(FULLTEXT)%

