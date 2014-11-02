# %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(tests)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensors)

## Declare ROS messages, services and actions
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(
# INCLUDE_DIRS include 
# Libraries
CATKIN_DEPENDS  sensors)

##Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( ${catkin_INCLUDE_DIRS})


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${sensors_INCLUDE_DIRS})


add_executable(demo_sub src/demo_sub.cpp)
target_link_libraries(demo_sub ${catkin_LIBRARIES})
add_dependencies(demo_sub sensors_gencpp)

add_executable(demo_sub_data src/demo_sub_data.cpp)
target_link_libraries(demo_sub_data ${catkin_LIBRARIES})
add_dependencies(demo_sub_data sensors_gencpp)

add_executable(demo_subM src/demo_subM.cpp)
target_link_libraries(demo_subM  ${catkin_LIBRARIES})
add_dependencies(demo_subM sensors_gencpp)

add_executable(demo_pub src/demo_pub.cpp)
target_link_libraries(demo_pub ${catkin_LIBRARIES})
add_dependencies(demo_pub sensors_gencpp)

add_executable(demo_pubM src/demo_pubM.cpp)
target_link_libraries(demo_pubM ${catkin_LIBRARIES})
add_dependencies(demo_pubM sensors_gencpp)

add_executable(motor_sub src/motor_sub.cpp)
target_link_libraries(motor_sub ${catkin_LIBRARIES})
add_dependencies(motor_sub sensors_gencpp std_msgs)

add_executable(accel_sub src/accel_sub.cpp)
target_link_libraries(accel_sub ${catkin_LIBRARIES})
add_dependencies(accel_sub sensors_gencpp std_msgs)

add_executable(paccel_sub src/paccel_sub.cpp)
target_link_libraries(paccel_sub ${catkin_LIBRARIES})
add_dependencies(paccel_sub sensors_gencpp std_msgs)

add_executable(command src/command.cpp)
target_link_libraries(command ${catkin_LIBRARIES})
add_dependencies(command geometry_msgs sensors_gencpp)

add_executable(test src/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES})
add_dependencies(test sensors_gencpp)

add_executable(minicube src/minicube.cpp ../sensors/src/robocomm.cpp)
target_link_libraries(minicube ${catkin_LIBRARIES})
add_dependencies(minicube sensors_gencpp)


#%EndTag(FULLTEXT)%


