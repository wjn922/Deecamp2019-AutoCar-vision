cmake_minimum_required(VERSION 2.8.3)
project(control)
 
find_package(catkin REQUIRED)
  


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

 
find_package(catkin REQUIRED COMPONENTS roscpp rospy)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amin
   CATKIN_DEPENDS  roscpp
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
)

add_executable(talker talker.cpp  )
target_link_libraries(talker  ${catkin_LIBRARIES})

