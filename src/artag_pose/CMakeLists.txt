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
    eigen_conversions
# !!!Hydro doesn't require it!!!----> : cmake_modules
)

# Set debug mode for breakpoint in gdb and QtCreator
# Comment for release
#set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-O3")        ## Optimize
endif()

find_package(Eigen REQUIRED)


# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/ArtagPose.cfg)

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
    src/tf_config_loader.cpp
    src/markers_pose.cpp
    src/particle_filter.cpp
    src/default_config.cpp

# PB: I added the header so they are visible in QtCreator,
# it's *NOT* require for compiling
    cfg/ArtagPose.cfg
    include/artag_pose/artag_pose_node.h
    include/artag_pose/artag_subscriber.h
    include/artag_pose/config_loader.h
    include/artag_pose/tf_config_loader.h
    include/artag_pose/markers_pose.h
    include/artag_pose/particle_filter.h
    include/artag_pose/default_config.h
)

add_dependencies(artag_pose_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(artag_pose_node ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencf)

## Specify libraries to link a library or executable target against
target_link_libraries(artag_pose_node
   yaml-cpp
   ${catkin_LIBRARIES}
)
