cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

# uncomment to use new solution which requires modified gazebo
# allowing enforcement of contact calculation in ContactManager
add_definitions("-DCONTACTS_ENFORCABLE")

# Search for dependencies
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(${PROJECT_SOURCE_DIR}/cmake/SearchForStuff.cmake)

set(collision_benchmark_HEADERS
  collision_benchmark/boost_std_conversion.hh
  collision_benchmark/BoostSerialization.hh
  collision_benchmark/ClientGui.hh
  collision_benchmark/ContactInfo.hh
  collision_benchmark/ControlServer.hh
  collision_benchmark/GazeboControlServer.hh
  collision_benchmark/GazeboHelpers.hh
  collision_benchmark/GazeboMultipleWorlds.hh
  collision_benchmark/GazeboMultipleWorldsServer.hh
  collision_benchmark/GazeboPhysicsWorld.hh
  collision_benchmark/GazeboStateCompare.hh
  collision_benchmark/GazeboTopicForwarder.hh
  collision_benchmark/GazeboTopicForwardingMirror.hh
  collision_benchmark/GazeboWorldLoader.hh
  collision_benchmark/GazeboWorldState.hh
  collision_benchmark/Helpers.hh
  collision_benchmark/MathHelpers.hh
  collision_benchmark/MathHelpers-inl.hh
  collision_benchmark/MirrorWorld.hh
  collision_benchmark/PhysicsWorld.hh
  collision_benchmark/PrimitiveShape.hh
  collision_benchmark/PrimitiveShapeParameters.hh
  collision_benchmark/Shape.hh
  collision_benchmark/SignalReceiver.hh
  collision_benchmark/SimpleTriMeshShape.hh
  collision_benchmark/TypeHelper.hh
  collision_benchmark/WorldManager.hh
)

add_library(collision_benchmark SHARED
  collision_benchmark/GazeboControlServer.cc
  collision_benchmark/GazeboHelpers.cc
  collision_benchmark/GazeboMultipleWorlds.cc
  collision_benchmark/GazeboMultipleWorldsServer.cc
  collision_benchmark/GazeboPhysicsWorld.cc
  collision_benchmark/GazeboStateCompare.cc
  collision_benchmark/GazeboTopicForwardingMirror.cc
  collision_benchmark/GazeboWorldLoader.cc
  collision_benchmark/GazeboWorldState.cc
  collision_benchmark/Helpers.cc
  collision_benchmark/MeshShapeGenerationVtk.cc
  collision_benchmark/PrimitiveShape.cc
  collision_benchmark/SignalReceiver.cc
  collision_benchmark/SimpleTriMeshShape.cc
  collision_benchmark/Shape.cc
  collision_benchmark/TypeHelper.cc
)

# expand the dependencies_* variables for the include directories and libraries
set(dependencies_INCLUDE_DIRS
  ${dependencies_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${PROJECT_SOURCE_DIR})

include_directories(${dependencies_INCLUDE_DIRS})
link_directories(${dependencies_LIBRARY_DIRS})

# when using a different folder for the header file, must do
# QT5_WRAP_CPP despite of the AUTOMOC=ON, or it won't generate the MOC
QT5_WRAP_CPP(ClientGui_MOC collision_benchmark/ClientGui.hh)
add_library(collision_benchmark_gui SHARED
    collision_benchmark/ClientGui.cc
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_custom_target(tests)

add_executable(multiple_worlds_server
  collision_benchmark/multiple_worlds_server.cc)

target_link_libraries(collision_benchmark
  ${dependencies_LIBRARIES})

target_link_libraries(collision_benchmark_gui
  ${dependencies_LIBRARIES})

target_link_libraries(multiple_worlds_server collision_benchmark)


#############################################
# messages
#############################################
find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set (msgs test/msgs/colliding_shapes.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(collision_benchmark_msgs SHARED ${PROTO_SRCS})
target_link_libraries(collision_benchmark_msgs ${PROTOBUF_LIBRARY})

# get a list of all generated headers
# set(PROTO_TEST_MSGS_HDRS)
# foreach(FIL ${msgs})
#   get_filename_component(ABS_FIL ${FIL} ABSOLUTE)
#   get_filename_component(FIL_WE ${FIL} NAME_WE)
#   list(APPEND PROTO_TEST_MSGS_HDRS "${CMAKE_CURRENT_BINARY_DIR}/${FIL_WE}.pb.h")
# endforeach()

#############################################
# testing
#############################################

# Compile the GUI library for the "colliding shapes test".
# When using a different folder for the header file, must to
# QT5_WRAP_CPP despite of the AUTOMOC=ON, or it won't generate the MOC
# -----------------------------------------
QT5_WRAP_CPP(CollidingShapesGui_MOC test/CollidingShapesGui.hh)
add_library(collision_benchmark_test_gui SHARED
    test/CollidingShapesGui.cc
    test/CollidingShapesParams.cc
)
target_link_libraries(collision_benchmark_test_gui collision_benchmark_msgs)
add_dependencies(tests collision_benchmark_test_gui)

QT5_WRAP_CPP(StepGui_MOC test/StepGui.hh)
add_library(collision_benchmark_step_gui SHARED
    test/StepGui.cc
)
target_link_libraries(collision_benchmark_step_gui
  ${dependencies_LIBRARIES})
add_dependencies(tests collision_benchmark_step_gui)


# compile testing libraries and executables
# -----------------------------------------

enable_testing()
# - Gtest build

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in ${CMAKE_BINARY_DIR}/googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
    RESULT_VARIABLE result
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

# names of both gtest libraries for linking
set(GTEST_BOTH_LIBRARIES gtest gtest_main)

set(TEST_LIB_SRCS
    test/TestUtils.cc
    test/MultipleWorldsTestFramework.cc
    test/StaticTestFramework.cc
    test/ContactsFlickerTestFramework.cc
    test/CollidingShapesTestFramework.cc
    test/CollidingShapesParams.cc)

add_library(collision_benchmark_test EXCLUDE_FROM_ALL ${TEST_LIB_SRCS})
target_link_libraries(collision_benchmark_test
  collision_benchmark ${GTEST_BOTH_LIBRARIES})
add_test(MultipleWorldsTest collision_benchmark_test)
add_dependencies(tests collision_benchmark_test)

add_executable(colliding_shapes_test
  test/colliding_shapes_test.cc)
target_link_libraries(colliding_shapes_test collision_benchmark
                                            collision_benchmark_test
                                            collision_benchmark_msgs)
add_dependencies(tests colliding_shapes_test)

add_executable(multiple_worlds_test EXCLUDE_FROM_ALL test/MultipleWorlds_TEST.cc)
target_link_libraries(multiple_worlds_test
  collision_benchmark collision_benchmark_test ${GTEST_BOTH_LIBRARIES})
add_test(MultipleWorldsTest multiple_worlds_test)
add_dependencies(tests multiple_worlds_test)

add_executable(world_interface_test EXCLUDE_FROM_ALL test/WorldInterface_TEST.cc)
target_link_libraries(world_interface_test
  collision_benchmark collision_benchmark_test ${GTEST_BOTH_LIBRARIES})
add_test(WorldInterfaceTest world_interface_test)
add_dependencies(tests world_interface_test)

add_executable(static_test EXCLUDE_FROM_ALL test/Static_TEST.cc)
target_link_libraries(static_test
  collision_benchmark collision_benchmark_test ${GTEST_BOTH_LIBRARIES})
add_test(StaticTest static_test)
add_dependencies(tests static_test)

add_executable(contacts_flicker_test EXCLUDE_FROM_ALL test/ContactsFlicker_TEST.cc)
target_link_libraries(contacts_flicker_test
  collision_benchmark collision_benchmark_test ${GTEST_BOTH_LIBRARIES})
add_test(StaticTest contacts_flicker_test)
add_dependencies(tests contacts_flicker_test)

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
  test_worlds/sphere_ode.world
  test_worlds/void.world)

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

install (TARGETS collision_benchmark multiple_worlds_server
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
