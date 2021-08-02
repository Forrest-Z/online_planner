cmake_minimum_required(VERSION 3.0.2)
project(online_planner)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "release")

find_package(cmake_modules REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(octomap REQUIRED)
find_package(dynamicEDT3D REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    visualization_msgs
    mavros_msgs
    message_generation
    depth2pc
    transform_utils
    gflags_catkin
    glog_catkin
    pcl_ros
    pcl_conversions
    traj_lib
    airsim_controller
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES online_planner
  CATKIN_DEPENDS geometry_msgs std_msgs roscpp visualization_msgs gflags_catkin glog_catkin depth2pc
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${DYNAMICEDT3D_INCLUDE_DIRS}
)

## Declare a C++ library
#add_library(online_planner
#   src/local_planner/motion_primitives_planner.cpp
#   src/mapping/octomap_handler.cpp
#   src/wrapper/base_wrapper.cpp
#)

#add_dependencies(online_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(parhp_node src/local_planner/motion_primitives_planner.cpp 
              src/mapping/octomap_handler.cpp
              src/wrapper/base_wrapper.cpp src/wrapper/parhp_wrapper.cpp)

add_dependencies(parhp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(parhp_node
   ${catkin_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${DYNAMICEDT3D_LIBRARIES}
   #online_planner
)
