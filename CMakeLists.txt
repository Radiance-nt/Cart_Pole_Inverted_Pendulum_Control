cmake_minimum_required(VERSION 3.15)
project(pendulum_control)

find_package(Eigen3 REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

add_compile_definitions(PROJECT_DIR="${PROJECT_SOURCE_DIR}")


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
#  LIBRARIES pendulum_control
#  CATKIN_DEPENDS roscpp
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
add_library(controller_libs
  src/controller_lqr.cpp
  src/controller_mpc.cpp
)
target_link_libraries(controller_libs
  OsqpEigen::OsqpEigen
)
add_executable(control src/control.cpp)
target_link_libraries(control
  controller_libs
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  OsqpEigen::OsqpEigen
)