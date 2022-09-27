# 'Tests' is the subproject name
project(Tests)

# 'lib' is the folder with Google Test sources
# add_subdirectory(googletest)

#add_subdirectory(googletest)
#include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(${GTEST_INCLUDE_DIRS})

add_subdirectory(rayTracerTests)



