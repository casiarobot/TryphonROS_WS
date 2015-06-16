
# %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(state)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs roscpp rospy std_msgs genmsg sensors tf 
# cmake_modules
)
find_package(Eigen REQUIRED)

## Declare ROS messages, services and actions
add_message_files(DIRECTORY msg FILES state.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs tf)


## Declare a catkin package
catkin_package( INCLUDE_DIRS
# Libraries
CATKIN_DEPENDS  sensor_msgs geometry_msgs tf
DEPENDS eigen)

##Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})


## Build talker and listener
include_directories( ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS})


add_executable(state_estimator src/state_estimator.cpp)
target_link_libraries(state_estimator ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(state_estimator sensors_gencpp std_msgs geometry_msgs)

add_executable(state_estimator_SICK src/state_estimator_SICK.cpp)
target_link_libraries(state_estimator_SICK ${catkin_LIBRARIES})
add_dependencies(state_estimator_SICK sensors_gencpp std_msgs geometry_msgs)

add_executable(state_sub src/state_sub.cpp)
target_link_libraries(state_sub ${catkin_LIBRARIES})
add_dependencies(state_sub sensors_gencpp std_msgs geometry_msgs)

add_executable(desired_state src/desired_state.cpp)
target_link_libraries(desired_state ${catkin_LIBRARIES})
## {state...} added to resolve issue with state.msg --> state.h in devel not being found
add_dependencies(desired_state  sensor_gencpp  geometry_msgs std_msgs ${state_EXPORTED_TARGETS})

#%EndTag(FULLTEXT)%

