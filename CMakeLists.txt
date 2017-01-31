cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

# uncomment to use new solution which requires modified gazebo
# allowing enforcement of contact calculation in ContactManager
# add_definitions("-DCONTACTS_ENFORCABLE")

# Search for dependencies
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(${PROJECT_SOURCE_DIR}/cmake/SearchForStuff.cmake)

set(collision_benchmark_HEADERS
  collision_benchmark/boost_std_conversion.hh
  collision_benchmark/ContactInfo.hh
  collision_benchmark/GazeboMirrorWorld.hh
  collision_benchmark/GazeboStateCompare.hh
  collision_benchmark/MirrorWorld.hh
  collision_benchmark/Shape.hh
  collision_benchmark/PrimitiveShape.hh
  collision_benchmark/PrimitiveShapeParameters.hh
  collision_benchmark/SimpleTriMeshShape.hh
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
  collision_benchmark/Shape.cc
  collision_benchmark/PrimitiveShape.cc
  collision_benchmark/SimpleTriMeshShape.cc
)
 
# when using a different folder for the header file, must to
# QT5_WRAP_CPP despite of the AUTOMOC=ON, or it won't generate the MOC
QT5_WRAP_CPP(ClientGui_MOC collision_benchmark/ClientGui.hh)
add_library(collision_benchmark_gui SHARED
    collision_benchmark/ClientGui.cc
)

# expand the dependencies_* variables for the include directories and libraries
set(dependencies_INCLUDE_DIRS
  ${dependencies_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR})

include_directories(${dependencies_INCLUDE_DIRS})
link_directories(${dependencies_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_custom_target(tests)

add_executable(multiple_worlds_server
  collision_benchmark/multiple_worlds_server.cc)

add_executable(multiple_worlds_server_test_simple
  collision_benchmark/multiple_worlds_server_test_simple.cc)

target_link_libraries(collision_benchmark
  ${dependencies_LIBRARIES})

target_link_libraries(collision_benchmark_gui
  ${dependencies_LIBRARIES})

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


add_executable(world_interface_test EXCLUDE_FROM_ALL test/WorldInterface_TEST.cc)
target_link_libraries(world_interface_test
  collision_benchmark ${GTEST_BOTH_LIBRARIES})
add_test(WorldInterfaceTest world_interface_test)
add_dependencies(tests world_interface_test)

# tutorials
add_custom_target(tutorials)

add_executable(transfer_world_state EXCLUDE_FROM_ALL tutorials/transfer_world_state.cc)
target_link_libraries(transfer_world_state collision_benchmark)
add_dependencies(tutorials transfer_world_state)

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
