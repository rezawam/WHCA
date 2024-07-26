cmake_minimum_required(VERSION 3.15)

# The project name
project(Cooperative_Pathfinding)

# detect operating system
message(STATUS "We are on a ${CMAKE_SYSTEM_NAME} system")

# set project version
set(CMAKE_CXX_STANDARD 17) 
set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 1)
set(${PROJECT_NAME}_VERSION "${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}")

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")


find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(include)

file(GLOB_RECURSE SOURCES "src/*.cpp")

# set(SOURCES main.cpp src/Solver.cpp include/Agent.hpp include/Graph.hpp include/Solver.hpp)
add_executable(${PROJECT_NAME} main.cpp ${SOURCES})
# target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} ${GTEST_LIBRARIES} pthread)

if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /W4 /WX)
else()
  target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

add_subdirectory(tests)