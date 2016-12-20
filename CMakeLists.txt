cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo 8.0 REQUIRED)

add_library(collision_benchmark SHARED
  collision_benchmark/WorldLoader.cc
  collision_benchmark/GazeboPhysicsWorld.cc
  #collision_benchmark/GazeboWorldState.cc
)

include_directories(${PROJECT_SOURCE_DIR} ${GAZEBO_INCLUDE_DIRS} ${DART_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_custom_target(tests)

add_executable(multiple_worlds_server
  collision_benchmark/multiple_worlds_server.cc)

target_link_libraries(collision_benchmark
  ${GAZEBO_LIBRARIES}
  ${DART_LIBRARIES})

target_link_libraries(multiple_worlds_server collision_benchmark)


#####################
# Testing
#####################

enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(multiple_worlds_test EXCLUDE_FROM_ALL test/MultipleWorlds_TEST.cc)
target_link_libraries(multiple_worlds_test
  collision_benchmark ${GTEST_BOTH_LIBRARIES})
add_test(MultipleWorldsTest multiple_worlds_test)
add_dependencies(tests multiple_worlds_test)
