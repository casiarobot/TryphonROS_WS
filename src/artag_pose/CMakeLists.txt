cmake_minimum_required(VERSION 2.8.3)
project(artag_pose)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin
REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    std_msgs
    dynamic_reconfigure
    sensor_msgs
    tf
    tf_conversions
  cmake_modules
)

# Set debug mode for breakpoint in gdb and QtCreator
# Comment for release
set(CMAKE_BUILD_TYPE Debug)


find_package(Eigen REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
	include
    CATKIN_DEPENDS
	geometry_msgs
	roscpp
	std_msgs
	dynamic_reconfigure
	sensor_msgs
	ar_track_alvar_msgs
	tf
	tf_conversions
    DEPENDS
	eigen
)


include_directories(
    include
    include/artag_pose
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)



## Declare a cpp executable
add_executable(artag_pose_node
    src/artag_pose_node.cpp
    src/artag_subscriber.cpp
    # src/yaml_config_loader.cpp
    src/tf_config_loader.cpp
    src/markers_pose.cpp
# PB: I added the header so they are visible in QtCreator,
# it's *NOT* require for compiling
    include/artag_pose/artag_pose_node.h
    include/artag_pose/artag_subscriber.h
    include/artag_pose/config_loader.h
    include/artag_pose/yaml_config_loader.h
    include/artag_pose/tf_config_loader.h
    include/artag_pose/markers_pose.h
)

add_executable(test_pose_node
    src/test_pose_node.cpp
# PB: I added the header so they are visible in QtCreator,
# it's *NOT* require for compiling
    include/artag_pose/test_pose_node.h

    config/cafeteria.yaml
)

add_dependencies(artag_pose_node ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencf)
add_dependencies(test_pose_node  ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencf)


## Specify libraries to link a library or executable target against
target_link_libraries(artag_pose_node
   yaml-cpp
   ${catkin_LIBRARIES}
)

target_link_libraries(test_pose_node
   yaml-cpp
   ${catkin_LIBRARIES}
)
