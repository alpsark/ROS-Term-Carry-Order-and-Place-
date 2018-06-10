cmake_minimum_required(VERSION 2.8.3)
project(pick_up_order)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf tf_conversions  image_transport visualization_msgs pcl_ros pcl_conversions vrep_common sensor_msgs
  cv_bridge)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vrepscan_to_laserscan
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( ${PCL_INCLUDE_DIRS})

include_directories (/usr/include/eigen3)

## Declare a C++ library
# add_library(ur10_kinematics_solution
#   src/${PROJECT_NAME}/ur10_kinematics_solution.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(ur10_kinematics_solution ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
# add_executable(ur10_kinematics_solution_node src/ur10_kinematics_solution_node.cpp)
add_executable(pick_up_order src/pick_up_order.cpp src/image_helper.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(ur10_kinematics_solution_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pick_up_order ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(ur10_kinematics_solution_node
#   ${catkin_LIBRARIES}
# )


target_link_libraries(pick_up_order ${catkin_LIBRARIES} orocos-kdl)
target_link_libraries(pick_up_order ${OpenCV_LIBRARIES})
target_link_libraries(pick_up_order ${PCL_LIBRARIES} ${Boost_LIBRARIES})
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
# install(TARGETS ur10_kinematics_solution ur10_kinematics_solution_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ur10_kinematics_solution.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
