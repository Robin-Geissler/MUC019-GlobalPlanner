cmake_minimum_required(VERSION 2.8.3)
project(global_planner)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  tf
  tufast_msgs
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_core
  CATKIN_DEPENDS
    nav_msgs
    tf
    tufast_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(global_planner
    src/global_planner_node.cpp
    src/global_planner.cpp
)

add_dependencies(global_planner
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(global_planner
  ${catkin_LIBRARIES}
)

