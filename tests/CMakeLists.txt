message(STATUS "Generating tests")

cmake_minimum_required(VERSION 3.15)
 
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
file(GLOB_RECURSE SOURCES "../src/*.cpp")

add_executable(runTests cases.cpp ${SOURCES})
target_link_libraries(runTests ${GTEST_LIBRARIES} pthread)