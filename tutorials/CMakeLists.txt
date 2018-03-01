cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

add_custom_target(tutorials)

add_executable(transfer_world_state EXCLUDE_FROM_ALL transfer_world_state.cc)
target_link_libraries(transfer_world_state collision_benchmark)
add_dependencies(tutorials transfer_world_state)

add_executable(multiple_worlds EXCLUDE_FROM_ALL multiple_worlds_server.cc)
target_link_libraries(multiple_worlds collision_benchmark)
add_dependencies(tutorials multiple_worlds)
