cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

set(ENABLE_TEST_INSTALL false)

# Compile the GUI library for the "colliding shapes test".
# When using a different folder for the header file, must to
# QT5_WRAP_CPP despite of the AUTOMOC=ON, or it won't generate the MOC
# -----------------------------------------
QT5_WRAP_CPP(CollidingShapesGui_MOC CollidingShapesGui.hh)
add_library(collision_benchmark_test_gui SHARED
    CollidingShapesGui.cc
    CollidingShapesParams.cc
)
target_link_libraries(collision_benchmark_test_gui collision_benchmark_msgs)
# add_dependencies(tests collision_benchmark_test_gui)

QT5_WRAP_CPP(StepGui_MOC StepGui.hh)
add_library(collision_benchmark_step_gui SHARED
    StepGui.cc
)
target_link_libraries(collision_benchmark_step_gui
  ${dependencies_LIBRARIES})
# add_dependencies(tests collision_benchmark_step_gui)


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
# the gtest and gtest_main targets. Installation of the targets
# can be switch on or off with the flags.
set(INSTALL_GTEST ${ENABLE_TEST_INSTALL} CACHE INTERNAL "Enable installation of googletest.")
set(INSTALL_GMOCK ${ENABLE_TEST_INSTALL} CACHE INTERNAL "Enable installation of googlemock.")
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
                 ${CMAKE_BINARY_DIR}/googletest-build)

# The ggtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

# names of both gtest libraries for linking
set(GTEST_BOTH_LIBRARIES gtest gtest_main)

set (collision_benchmark_test_HEADERS
  BasicTestFramework.hh
  MultipleWorldsTestFramework.hh
)

set(collision_benchmark_test_SRCS
    TestUtils.cc
    MultipleWorldsTestFramework.cc
    StaticTestFramework.cc
    ContactsFlickerTestFramework.cc
    CollidingShapesTestFramework.cc
    CollidingShapesParams.cc)

add_library(collision_benchmark_test
  EXCLUDE_FROM_ALL ${collision_benchmark_test_SRCS})
target_link_libraries(collision_benchmark_test
  collision_benchmark ${GTEST_BOTH_LIBRARIES})
add_test(MultipleWorldsTest collision_benchmark_test)
#add_dependencies(tests collision_benchmark_test)

add_executable(colliding_shapes_test colliding_shapes_test.cc)
target_link_libraries(colliding_shapes_test collision_benchmark
                                            collision_benchmark_test
                                            collision_benchmark_msgs)
add_dependencies(tests colliding_shapes_test)

add_executable(multiple_worlds_test EXCLUDE_FROM_ALL MultipleWorlds_TEST.cc)
target_link_libraries(multiple_worlds_test
  collision_benchmark collision_benchmark_test ${GTEST_BOTH_LIBRARIES})
add_test(MultipleWorldsTest multiple_worlds_test)
add_dependencies(tests multiple_worlds_test)

add_executable(world_interface_test EXCLUDE_FROM_ALL WorldInterface_TEST.cc)
target_link_libraries(world_interface_test
  collision_benchmark collision_benchmark_test ${GTEST_BOTH_LIBRARIES})
add_test(WorldInterfaceTest world_interface_test)
add_dependencies(tests world_interface_test)

add_executable(static_test EXCLUDE_FROM_ALL Static_TEST.cc)
target_link_libraries(static_test
  collision_benchmark collision_benchmark_test ${GTEST_BOTH_LIBRARIES})
add_test(StaticTest static_test)
add_dependencies(tests static_test)

add_executable(contacts_flicker_test EXCLUDE_FROM_ALL ContactsFlicker_TEST.cc)
target_link_libraries(contacts_flicker_test
  collision_benchmark collision_benchmark_test ${GTEST_BOTH_LIBRARIES})
add_test(StaticTest contacts_flicker_test)
add_dependencies(tests contacts_flicker_test)

#############################
# Installation
#############################
if (ENABLE_TEST_INSTALL)
  install (FILES ${collision_benchmark_test_HEADERS}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/collision_benchmark/test/)

  install (TARGETS
      collision_benchmark_test
      collision_benchmark_test_gui
      collision_benchmark_step_gui
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif (ENABLE_TEST_INSTALL)
