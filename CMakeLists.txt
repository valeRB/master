cmake_minimum_required(VERSION 2.8.3)
project(master)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  robot_msgs
  actionlib
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES master
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(move_master src/move_master.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(move_master move_master_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(move_master ${catkin_LIBRARIES})
