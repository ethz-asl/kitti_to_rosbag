cmake_minimum_required(VERSION 2.8.3)
project(kitti_to_rosbag)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/kitti_parser.cpp
)

############
# BINARIES #
############
#cs_add_executable(voxblox_rox
#  src/voxblox.cc
#)
#target_link_libraries(voxblox_rox ${PROJECT_NAME})


##########
# EXPORT #
##########
cs_install()
cs_export()
