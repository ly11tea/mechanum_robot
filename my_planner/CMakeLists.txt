cmake_minimum_required(VERSION 2.8.3)
project(my_planner)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  laser_geometry
  visualization_msgs
  topic_tools
  move_base_msgs
  actionlib
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
#  LIBRARIES my_planner
  CATKIN_DEPENDS geometry_msgs roscpp tf  laser_geometry visualization_msgs topic_tools move_base_msgs actionlib
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
# add_library(my_planner
#   src/${PROJECT_NAME}/my_planner.cpp
# )

## Declare a cpp executable
# add_executable(my_planner_node src/my_planner_main.cpp)
# add_executable(my_planner_node1 src/my_planner_main1.cpp)
# add_executable(my_planner_node2 src/my_planner_main2.cpp)
# add_executable(my_planner_node3 src/my_planner_main3.cpp)
# add_executable(my_planner_node4 src/my_planner_main4.cpp)
# add_executable(my_planner_node4_p src/my_planner_main4_p.cpp)
# add_executable(my_planner_node5 src/my_planner_main5.cpp)
# add_executable(my_planner_node6 src/my_planner_main6.cpp)
# add_executable(my_planner_node7 src/my_planner_main7.cpp)
# add_executable(my_planner_node8 src/my_planner_main8.cpp)
# add_executable(my_planner_node9 src/my_planner_main9.cpp)
# add_executable(my_planner_node10 src/my_planner_main10.cpp)
# add_executable(my_planner_node11 src/my_planner_main11.cpp)

#  add_executable(my_planner_change_topic_node src/my_planner_change_topic_main.cpp)
#  add_executable(my_planner_main12_test_node src/my_planner_main12_test.cpp)
#  add_executable(my_planner_main13_test_node src/my_planner_main13_test.cpp)  
#  add_executable(my_planner_main14_test_node src/my_planner_main14_test.cpp) 
#  add_executable(my_planner_main15_test_pid_node src/my_planner_main15_test_pid.cpp) 
#  add_executable(my_planner_main16_test_pid_boost_node src/my_planner_main16_test_pid_boost.cpp src/PID_Beta7.cpp) 
  add_executable(my_planner_main17_test_pid_node src/my_planner_main17_test_pid.cpp) 
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(my_planner_node my_planner_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(my_planner_node
 #  ${catkin_LIBRARIES}
# )
 
#  target_link_libraries(my_planner_node1
 #  ${catkin_LIBRARIES}
# )
 
#   target_link_libraries(my_planner_node2
#   ${catkin_LIBRARIES}
# )
 
#    target_link_libraries(my_planner_node3
#   ${catkin_LIBRARIES}
# )
 
#   target_link_libraries(my_planner_node4
#   ${catkin_LIBRARIES}
# )
 
#    target_link_libraries(my_planner_node4_p
#   ${catkin_LIBRARIES}
# )
#   target_link_libraries(my_planner_node5
#   ${catkin_LIBRARIES}
# )
 
#    target_link_libraries(my_planner_node6
 #  ${catkin_LIBRARIES}
# )
 
#     target_link_libraries(my_planner_node7
#   ${catkin_LIBRARIES}
# )

 #    target_link_libraries(my_planner_node8
#   ${catkin_LIBRARIES}
# )
#     target_link_libraries(my_planner_node9
#   ${catkin_LIBRARIES}
# )

 #    target_link_libraries(my_planner_node10
 #  ${catkin_LIBRARIES}
 #)
 
 #     target_link_libraries(my_planner_node11
#   ${catkin_LIBRARIES}
# )
 
 #    target_link_libraries(my_planner_change_topic_node 
 #  ${catkin_LIBRARIES}
# )
#     target_link_libraries( my_planner_main12_test_node 
#   ${catkin_LIBRARIES}
# )
 
#    target_link_libraries(my_planner_main13_test_node 
#   ${catkin_LIBRARIES}
# )
 
#     target_link_libraries(my_planner_main14_test_node 
#   ${catkin_LIBRARIES}
# )
 
#      target_link_libraries(my_planner_main15_test_pid_node 
#   ${catkin_LIBRARIES}
# )

#     target_link_libraries(my_planner_main16_test_pid_boost_node
#  ${Boost_LIBRARIES}
#   ${catkin_LIBRARIES}
# )

     target_link_libraries(my_planner_main17_test_pid_node
  ${Boost_LIBRARIES}
   ${catkin_LIBRARIES}
 )


 
 
