cmake_minimum_required(VERSION 2.8.3)
project(mechanum_control_odom)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mechanum_control_odom
   CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(mechanum_control_odom
#   src/${PROJECT_NAME}/mechanum_control_odom.cpp
# )

## Declare a cpp executable
 add_executable(mechanum_control_odom_node src/mechanum_control_odom_main.cpp src/SerialPort.cpp)
 add_executable(mechanum_control_odom_node1 src/mechanum_control_odom_main1.cpp src/SerialPort.cpp)
 add_executable(mechanum_control_odom_node2 src/mechanum_control_odom_main2.cpp src/SerialPort.cpp)
 
 add_executable(mechanum_control_odom_node3 src/mechanum_control_odom_main3.cpp src/SerialPort.cpp)
 
 add_executable(mechanum_control_odom_median_filter src/mechanum_control_odom_median_filter.cpp src/SerialPort.cpp)
 
  add_executable(mechanum_control_odom_average_filter src/mechanum_control_odom_average_filter.cpp src/SerialPort.cpp)
  
  add_executable(mechanum_control_odom_tf_main src/mechanum_control_odom_tf_main.cpp src/SerialPort.cpp)
  
  add_executable(mechanum_control_node src/mechanum_control.cpp src/SerialPort.cpp)  
  add_executable(odom_test src/odom_test.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(mechanum_control_odom_node mechanum_control_odom_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(mechanum_control_odom_node
  ${catkin_LIBRARIES}
 )
 
  target_link_libraries(mechanum_control_odom_node1
  ${catkin_LIBRARIES}
 )
 
 target_link_libraries(mechanum_control_odom_node2
  ${catkin_LIBRARIES}
 )

 target_link_libraries(mechanum_control_odom_node3
  ${catkin_LIBRARIES}
 )
 
  target_link_libraries(mechanum_control_odom_median_filter
  ${catkin_LIBRARIES}
 )
 
   target_link_libraries(mechanum_control_odom_average_filter
  ${catkin_LIBRARIES}
 )
 
    target_link_libraries(mechanum_control_odom_tf_main
  ${catkin_LIBRARIES}
 )
 
     target_link_libraries(mechanum_control_node
  ${catkin_LIBRARIES}
 )
 
     target_link_libraries(odom_test
  ${catkin_LIBRARIES}
 )

 
