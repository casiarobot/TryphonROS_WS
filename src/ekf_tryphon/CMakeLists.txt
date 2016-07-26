cmake_minimum_required(VERSION 2.8.3)
project(ekf_tryphon)


set(CMAKE_BUILD_TYPE Debug)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs roscpp rospy std_msgs genmsg sensors tf dynamic_reconfigure
 cmake_modules
)
find_package(Eigen REQUIRED)

## Declare ROS messages, services and actions
#add_message_files(DIRECTORY msg FILES state.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs tf)

generate_dynamic_reconfigure_options(
  cfg/ekf.cfg
)

## Declare a catkin package
catkin_package( INCLUDE_DIRS include
# Libraries
CATKIN_DEPENDS  sensor_msgs geometry_msgs tf dynamic_reconfigure
DEPENDS eigen)

##Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})



## Build talker and listener
include_directories( ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS})


## Declare a C++ executable
#add_executable(kalmanfilter
#src/kalmanfilter.cpp
#src/kalmanbasefilter.cpp
#include/kalmanbasefilter.h
#)
#add_dependencies(kalmanfilter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(kalmanfilter ${catkin_LIBRARIES})

## Declare a C++ executable
add_executable(kalmanmeilleur
src/kalmanmeilleur.cpp
src/kalmanbasefilter.cpp
include/kalmanbasefilter.h
)

## Declare a C++ executable
add_executable(kalmanmeilleur_dock
src/kalmanbasefilter.cpp
src/kalmanmeilleur_dock.cpp
include/kalmanbasefilter.h
)

add_executable(kalmanmeilleur_dock_target
src/kalmanbasefilter.cpp
src/kalmanmeilleur_dock_target.cpp
include/kalmanbasefilter.h
)

add_executable(ar_temp
src/ar_temp.cpp)

add_executable(ar_temp_target
src/ar_temp_target.cpp)

add_dependencies(kalmanmeilleur ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(kalmanmeilleur ${catkin_LIBRARIES})

add_dependencies(kalmanmeilleur_dock ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(kalmanmeilleur_dock ${catkin_LIBRARIES})

add_dependencies(ar_temp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(ar_temp ${catkin_LIBRARIES})

add_dependencies(kalmanmeilleur_dock_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(kalmanmeilleur_dock_target ${catkin_LIBRARIES})

add_dependencies(ar_temp_target ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
target_link_libraries(ar_temp_target ${catkin_LIBRARIES})
