cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo 8.0 REQUIRED)


# Qt required for GUI plugin
find_package (Qt4)
#find_package(Protobuf REQUIRED)
include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})



set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

add_library(collision_benchmark SHARED
  collision_benchmark/WorldLoader.cc
  collision_benchmark/GazeboPhysicsWorld.cc
  collision_benchmark/GazeboWorldState.cc
  collision_benchmark/GazeboMirrorWorld.cc
  collision_benchmark/GazeboHelpers.cc
  collision_benchmark/GazeboStateCompare.cc
)

QT4_WRAP_CPP(headers_MOC collision_benchmark/ClientGui.hh)
add_library(collision_benchmark_gui SHARED
  ${headers_MOC}
  collision_benchmark/ClientGui.cc
)


include_directories(${PROJECT_SOURCE_DIR} ${GAZEBO_INCLUDE_DIRS} ${DART_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_custom_target(tests)

add_executable(multiple_worlds_server_test
  collision_benchmark/multiple_worlds_server_test.cc)

add_executable(multiple_worlds_server_test_simple
  collision_benchmark/multiple_worlds_server_test_simple.cc)

target_link_libraries(collision_benchmark
  ${GAZEBO_LIBRARIES}
  ${DART_LIBRARIES})

target_link_libraries(collision_benchmark_gui
  ${QT_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${DART_LIBRARIES})

target_link_libraries(multiple_worlds_server_test_simple collision_benchmark)
target_link_libraries(multiple_worlds_server_test collision_benchmark)

# testing
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(multiple_worlds_test EXCLUDE_FROM_ALL test/MultipleWorlds_TEST.cc)
target_link_libraries(multiple_worlds_test
  collision_benchmark ${GTEST_BOTH_LIBRARIES})
add_test(MultipleWorldsTest multiple_worlds_test)
add_dependencies(tests multiple_worlds_test)
