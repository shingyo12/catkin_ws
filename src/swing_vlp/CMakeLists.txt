cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 11)
project(swing_vlp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pcl_conversions
  PCL
)

find_package(CUDA 8.0 REQUIRED)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch sm_62)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
#  INCLUDE_DIRS include
#  LIBRARIES swing_vlp
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/swing_vlp.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/swing_vlp_node.cpp)

add_executable(usb_cycle  src/usb_cycle.cpp)
add_executable(usb_gps  src/usb_gps.cpp)
add_executable(usb_gps_js  src/usb_gps_js.cpp)
add_executable(usb_gps0_js  src/usb_gps0_js.cpp)
add_executable(usb_gps1_js  src/usb_gps1_js.cpp)
add_executable(vlp_pcd  src/vlp_pcd.cpp)

add_executable(vlp_pcd_imu  src/vlp_pcd_imu.cpp)
add_executable(sub_pcl_visualizer  src/sub_pcl_visualizer.cpp)
add_executable(pcd_record  src/pcd_record.cpp)
add_executable(manual_move  src/manual_move.cpp)
add_executable(manual_icp  src/manual_icp.cpp)
add_executable(manual_icp2  src/manual_icp2.cpp)
add_executable(manual_icp_multi  src/manual_icp_multi.cpp)
add_executable(manual_visualizer  src/manual_visualizer.cpp)
add_executable(manual_visualizer2  src/manual_visualizer2.cpp)
add_executable(manual_visualizer3  src/manual_visualizer3.cpp)
add_executable(manual_filter  src/manual_filter.cpp)
add_executable(manual_gps  src/manual_gps.cpp)
add_executable(manual_transform  src/manual_transform.cpp)
add_executable(manual_vgf  src/manual_vgf.cpp)
add_executable(rr_icp  src/rr_icp.cpp)
add_executable(usb_imu  src/usb_imu.cpp)
add_executable(map_make  src/map_make.cpp)
add_executable(swing_core  src/swing_core.cpp)
add_executable(pose  src/pose.cpp)
add_executable(tri_gps  src/tri_gps.cpp)

set(MIN_CUDA_SRC
  src/gpu/vlp_gpu.cpp
  src/gpu/swing_move.cu
)

cuda_add_executable(vlp_gpu
  ${MIN_CUDA_SRC}
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

target_link_libraries(usb_cycle   ${catkin_LIBRARIES})
target_link_libraries(usb_gps   ${catkin_LIBRARIES})
target_link_libraries(usb_gps_js  ${catkin_LIBRARIES})
target_link_libraries(usb_gps0_js  ${catkin_LIBRARIES})
target_link_libraries(usb_gps1_js  ${catkin_LIBRARIES})
target_link_libraries(vlp_pcd  ${catkin_LIBRARIES})

target_link_libraries(vlp_pcd_imu   ${catkin_LIBRARIES})
target_link_libraries(sub_pcl_visualizer   ${catkin_LIBRARIES})
target_link_libraries(pcd_record  ${catkin_LIBRARIES})
target_link_libraries(manual_move   ${catkin_LIBRARIES})
target_link_libraries(manual_icp   ${catkin_LIBRARIES})
target_link_libraries(manual_icp2   ${catkin_LIBRARIES})
target_link_libraries(manual_icp_multi   ${catkin_LIBRARIES})
target_link_libraries(manual_visualizer   ${catkin_LIBRARIES})
target_link_libraries(manual_visualizer2   ${catkin_LIBRARIES})
target_link_libraries(manual_visualizer3   ${catkin_LIBRARIES})
target_link_libraries(manual_filter   ${catkin_LIBRARIES})
target_link_libraries(manual_gps   ${catkin_LIBRARIES})
target_link_libraries(manual_transform   ${catkin_LIBRARIES})
target_link_libraries(manual_vgf   ${catkin_LIBRARIES})
target_link_libraries(rr_icp   ${catkin_LIBRARIES})
target_link_libraries(usb_imu   ${catkin_LIBRARIES})
target_link_libraries(map_make   ${catkin_LIBRARIES})
target_link_libraries(vlp_gpu   ${catkin_LIBRARIES})
target_link_libraries(swing_core   ${catkin_LIBRARIES})
target_link_libraries(pose   ${catkin_LIBRARIES})
target_link_libraries(tri_gps   ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_swing_vlp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

#target_link_libraries(vlp_gpu
#  ${catkin_LIBRARIES}
#  ${CUDA_LIBRARIES}
#)
