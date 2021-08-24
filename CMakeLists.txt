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
    vins_vio_mod
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
add_library(${PROJECT_NAME} SHARED
   src/local_planner/motion_primitives_planner.cpp
   src/mapping/octomap_handler.cpp
   src/mapping/featuremap_handler.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${EIGEN3_LIBS})

## Declare a C++ executable
add_executable(parhp_node src/wrapper/parhp_wrapper.cpp src/wrapper/base_wrapper.cpp)
add_dependencies(parhp_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(parhp_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
)

add_executable(maptest_node src/wrapper/maptest_wrapper.cpp src/wrapper/base_wrapper.cpp)
add_dependencies(maptest_node ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(maptest_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
)


#### tests
add_executable(qh_test src/test/qh_test.cpp)
add_dependencies(qh_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(qh_test ${catkin_LIBRARIES})