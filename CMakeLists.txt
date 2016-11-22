cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo 8.0 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
  
add_library(collision_benchmark SHARED
  collision_benchmark/WorldLoader.cc
)

add_executable(multiple_worlds_server
  collision_benchmark/multiple_worlds_server.cc)

target_link_libraries(collision_benchmark ${GAZEBO_LIBRARIES} pthread)
target_link_libraries(multiple_worlds_server collision_benchmark) 
