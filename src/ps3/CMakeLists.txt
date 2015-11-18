# %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(ps3)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs roscpp rospy std_msgs state)

## Declare ROS messages, services and actions
#add_message_files(DIRECTORY msg FILES sonar.msg sonarArray.msg compass.msg motor.msg motorArray.msg forces.msg imuros.msg state.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
# Libraries
CATKIN_DEPENDS   )

##Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( ${catkin_INCLUDE_DIRS})

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(ps3 src/ps3.cpp)
target_link_libraries(ps3 ${catkin_LIBRARIES})
add_dependencies(ps3 sensors_gencpp  sensor_msgs geometry_msgs)

add_executable(ps3_z src/ps3_z.cpp)
target_link_libraries(ps3_z ${catkin_LIBRARIES})
add_dependencies(ps3_z sensors_gencpp std_msgs  geometry_msgs state)

add_executable(ps3XZLC src/ps3XZLC.cpp)
target_link_libraries(ps3XZLC ${catkin_LIBRARIES})
add_dependencies(ps3XZLC sensors_gencpp std_msgs  geometry_msgs state)

#%EndTag(FULLTEXT)%

