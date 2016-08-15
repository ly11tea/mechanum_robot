cmake_minimum_required(VERSION 2.8.3)
project(call_clear_costmap_srv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES call_clear_costmap_srv
  CATKIN_DEPENDS roscpp std_msgs std_srvs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(call_clear_costmap_srv
#   src/${PROJECT_NAME}/call_clear_costmap_srv.cpp
# )

## Declare a cpp executable
 add_executable(call_clear_costmap_srv_node src/call_clear_costmap_srv_main.cpp)

 
## Specify libraries to link a library or executable target against
 target_link_libraries(call_clear_costmap_srv_node
   ${catkin_LIBRARIES}
 )


