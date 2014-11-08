# %Tag(FULLTEXT)%

cmake_minimum_required(VERSION 2.8.3)
project(controls)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy std_msgs sensors state dynamic_reconfigure)
find_package(Eigen REQUIRED)

## Declare ROS messages, services and actions 
#add_message_files(DIRECTORY msg FILES sonar.msg sonarArray.msg compass.msg motor.msg motorArray.msg forces.msg imuros.msg state.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs)

generate_dynamic_reconfigure_options(
  cfg/control.cfg
)

## Declare a catkin package
catkin_package( INCLUDE_DIRS include
# Libraries
CATKIN_DEPENDS  sensor_msgs geometry_msgs sensors state
DEPENDS eigen)

##Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( ${catkin_INCLUDE_DIRS})


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS})


add_executable(control src/control.cpp)
target_link_libraries(control ${catkin_LIBRARIES})
add_dependencies(control sensors_gencpp std_msgs geometry_msgs sensors state)
add_executable(ctr_fuzzy src/ctr_fuzzy.cpp)
target_link_libraries(ctr_fuzzy ${catkin_LIBRARIES})
add_dependencies(ctr_fuzzy sensors_gencpp std_msgs geometry_msgs sensors state)

#%EndTag(FULLTEXT)%

