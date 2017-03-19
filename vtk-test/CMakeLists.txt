cmake_minimum_required(VERSION 2.8)
 
PROJECT(TestPrimitives)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
 
add_executable(TestPrimitives test_primitives.cc)
add_definitions(-g)

if(VTK_LIBRARIES)
  target_link_libraries(TestPrimitives ${VTK_LIBRARIES})
else()
  target_link_libraries(TestPrimitives vtkHybrid vtkWidgets)
endif()
