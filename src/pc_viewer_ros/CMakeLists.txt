cmake_minimum_required(VERSION 2.8.3)
project(pc_viewer_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pcl_conversions pcl_ros roscpp sensor_msgs)
#find_package(PCL REQUIRED COMPONENTS segmentation filters visualization) # NON CATKIN DEPENDENT

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES pc_viewer_ros
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
system_lib
)

## Declare a cpp library
# add_library(pc_viewer_ros
#   src/${PROJECT_NAME}/pc_viewer_ros.cpp
# )

## Declare a cpp executable
 add_executable(pc_viewer_ros_node src/pc_viewer_ros_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(pc_viewer_ros_node pc_viewer_ros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(pc_viewer_ros_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARY_DIRS}
#   ${PCL_LIBRARY_DIRS} # NON CATKIN DEPENDENT
 )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables and/or libraries for installation
 install(TARGETS pc_viewer_ros_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

### NON CATKIN DEPENDENT
# install(TARGETS pc_viewer_ros_node
#   ARCHIVE DESTINATION $ENV{CMAKE_INSTALL_PREFIX}/lib
#   LIBRARY DESTINATION $ENV{CMAKE_INSTALL_PREFIX}/lib
#   RUNTIME DESTINATION $ENV{CMAKE_INSTALL_PREFIX}/bin
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pc_viewer_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

