cmake_minimum_required(VERSION 3.15)

# The project name
project(Coopirative_Pathfinding)

# detect operating system
message(STATUS "We are on a ${CMAKE_SYSTEM_NAME} system")

# set project version
set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 1)
set(${PROJECT_NAME}_VERSION "${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}")

include_directories(include)

file(GLOB_RECURSE SOURCES "src/*.cpp")

# set(SOURCES main.cpp src/Solver.cpp include/Agent.hpp include/Graph.hpp include/Solver.hpp)
add_executable(${PROJECT_NAME} main.cpp ${SOURCES})
# target_include_directories(${PROJECT_NAME} PRIVATE include)