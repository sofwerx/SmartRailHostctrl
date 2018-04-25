cmake_minimum_required(VERSION 2.8.3)
project(smartrail_hostctrl)

## The following option is necessary for YouCompleteMe to build its 
#   compiler database
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  std_msgs
  rosserial_msgs
  roslaunch
  topic_tools
  message_generation
)

# check for common flaws
roslaunch_add_file_check(launch)

find_package(Boost REQUIRED COMPONENTS 
  system
  thread
)

###################################
## message generation ##
###################################
add_message_files(
  FILES
  PgsDirectControl.msg
  PgsCorrection.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
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
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
)
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(pgs_node src/pgs_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pgs_node
   ${catkin_LIBRARIES}
)

## set target properties to help clarify output
set_target_properties(pgs_node PROPERTIES OUTPUT_NAME pgs_node PREFIX "")

add_executable(pgs_stream_node src/pgs_stream_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(pgs_stream_node
   ${catkin_LIBRARIES}
)
## set target properties to help clarify output
set_target_properties(pgs_stream_node PROPERTIES OUTPUT_NAME pgs_stream_node PREFIX "")

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
	add_subdirectory(tests)
endif()
