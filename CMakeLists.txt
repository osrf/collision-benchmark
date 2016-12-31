cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo 8.0 REQUIRED)


# Qt required for GUI plugin
find_package (Qt4)
#find_package(Protobuf REQUIRED)
include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

set(collision_benchmark_HEADERS
  collision_benchmark/boost_std_conversion.hh
  collision_benchmark/ContactInfo.hh
  collision_benchmark/GazeboMirrorWorld.hh
  collision_benchmark/GazeboStateCompare.hh
  collision_benchmark/MirrorWorld.hh
  collision_benchmark/Shape.hh
  collision_benchmark/WorldManager.hh
  collision_benchmark/ClientGui.hh
  collision_benchmark/GazeboHelpers.hh
  collision_benchmark/GazeboPhysicsWorld.hh
  collision_benchmark/GazeboWorldState.hh
  collision_benchmark/PhysicsWorld.hh
  collision_benchmark/GazeboWorldLoader.hh
)

add_library(collision_benchmark SHARED
  collision_benchmark/GazeboWorldLoader.cc
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

add_executable(multiple_worlds_server
  collision_benchmark/multiple_worlds_server.cc)

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
target_link_libraries(multiple_worlds_server collision_benchmark)

# testing
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(multiple_worlds_test EXCLUDE_FROM_ALL test/MultipleWorlds_TEST.cc)
target_link_libraries(multiple_worlds_test
  collision_benchmark ${GTEST_BOTH_LIBRARIES})
add_test(MultipleWorldsTest multiple_worlds_test)
add_dependencies(tests multiple_worlds_test)

add_executable(interface_tests EXCLUDE_FROM_ALL test/WorldInterface_TEST.cc)
target_link_libraries(interface_tests
  collision_benchmark ${GTEST_BOTH_LIBRARIES})
add_test(WorldInterfaceTest interface_tests)
add_dependencies(tests interface_tests)



# installing
set (test_WORLDS
  test_worlds/cube_bullet.world
  test_worlds/cube_dart.world
  test_worlds/cube_ode.world
  test_worlds/empty_bullet.world
  test_worlds/empty_dart.world
  test_worlds/empty_ode.world
  test_worlds/sphere_bullet.world
  test_worlds/sphere_dart.world
  test_worlds/sphere_ode.world)

set(physics_SDF
  physics_settings/bullet_default.sdf
  physics_settings/dart_default.sdf
  physics_settings/ode_default.sdf)

install (FILES ${collision_benchmark_HEADERS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/collision_benchmark)
install (FILES ${physics_SDF}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/physics_settings)
install (FILES ${test_WORLDS}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/test_worlds)

install (TARGETS collision_benchmark multiple_worlds_server multiple_worlds_server_test_simple
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
