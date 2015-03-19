# %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(tests)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensors sensor_msgs message_generation)

## Declare ROS messages, services and actions
#add_service_files(FILES AddTwoInts.srv)

add_message_files(DIRECTORY msg FILES props_command.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
# INCLUDE_DIRS include
# Libraries
CATKIN_DEPENDS  message_runtime sensors)

##Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( ${catkin_INCLUDE_DIRS})


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${sensors_INCLUDE_DIRS})


#add_executable(demo_sub_data src/demo_sub_data.cpp)
#target_link_libraries(demo_sub_data ${catkin_LIBRARIES})
#add_dependencies(demo_sub_data sensors_gencpp)

#add_executable(demo_sub_data_gazebo src/demo_sub_data_gazebo.cpp)
#target_link_libraries(demo_sub_data_gazebo ${catkin_LIBRARIES})
#add_dependencies(demo_sub_data_gazebo sensors_gencpp)

add_executable(data_recorder src/data_recorder.cpp)
target_link_libraries(data_recorder ${catkin_LIBRARIES})
add_dependencies(data_recorder sensors_gencpp)


add_executable(command src/command.cpp)
target_link_libraries(command ${catkin_LIBRARIES})
add_dependencies(command geometry_msgs sensors_gencpp)

add_executable(command_gazebo src/command_gazebo.cpp)
target_link_libraries(command_gazebo ${catkin_LIBRARIES})
add_dependencies(command_gazebo geometry_msgs sensors_gencpp)

add_executable(test src/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES})
add_dependencies(test sensors_gencpp)

add_executable(test_motors src/test_motors.cpp)
target_link_libraries(test_motors ${catkin_LIBRARIES})
add_dependencies(test_motors sensors_gencpp)

#add_executable(minicube src/minicube.cpp ../sensors/src/robocomm.cpp)
#target_link_libraries(minicube ${catkin_LIBRARIES})
#add_dependencies(minicube sensors_gencpp)


#%EndTag(FULLTEXT)%


