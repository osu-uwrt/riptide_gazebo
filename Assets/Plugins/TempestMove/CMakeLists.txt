cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project("Tempest_Move_Plugin")

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMKAE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(Tempest_Move_Plugin SHARED TempestMovePlugin.cpp)
target_link_libraries(Tempest_Move_Plugin ${GAZEBO_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")