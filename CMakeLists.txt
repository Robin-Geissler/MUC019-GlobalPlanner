cmake_minimum_required(VERSION 2.8.3)
project(global_planner)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  tf
  tufast_msgs
  jsk_recognition_msgs
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
    jsk_recognition_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


# global planner Node
add_executable(global_planner
    include/global_planner_node.hpp
    src/global_planner_node.cpp
    include/rayTracer.hpp
    src/rayTracer.cpp)

add_dependencies(global_planner
        ${catkin_EXPORTED_TARGETS}
        )

target_link_libraries(global_planner
        ${catkin_LIBRARIES}
        )

# Tests for the global Planner
add_subdirectory(tests)

# rayTracer Tests
add_executable(rayTracerTests
        tests/rayTracerTests/rayTracerTests.cpp
        include/rayTracer.hpp
        src/rayTracer.cpp
        )

target_link_libraries(rayTracerTests gtest gmock  pthread)



