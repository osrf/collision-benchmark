#################################################
# Qt required for GUI plugin
find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)
set(CMAKE_AUTOMOC ON)

#################################################
# Find tinyxml. Only required to check for SDF validity.
find_package(tinyxml REQUIRED)

#################################################
# Find Gazebo
find_package(gazebo 10.0 REQUIRED)

#################################################
# Find Assimp
find_package(assimp REQUIRED)

#################################################
# find VTK
# required helper for generation of meshes for primitive shapes
find_package(VTK REQUIRED)
# This should ideally be included but leads to compile errors.
#include(${VTK_USE_FILE})
#message("################################ ${VTK_USE_FILE}")

if(VTK_LIBRARIES)
  set(VTK_LIBS ${VTK_LIBRARIES})
else()
  set(VTK_LIBS vtkHybrid vtkWidgets)
endif()

#################################################
# find boost incl. serialization
find_package(Boost REQUIRED COMPONENTS system filesystem serialization)

#################################################
# Set variables with all dependencies
set(dependencies_INCLUDE_DIRS
  ${ASSIMP_INCLUDE_DIRS}
  ${assimp_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR}
  ${VTK_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(dependencies_LIBRARY_DIRS
  ${ASSIMP_LIBRARY_DIRS}
  ${assimp_LIBRARY_DIRS}
  ${TINYXML_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${VTK_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

set(dependencies_LIBRARIES
  ${ASSIMP_LIBRARIES}
  ${assimp_LIBRARIES}
  ${TINYXML_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES}
  ${VTK_LIBS}
  ${Boost_LIBRARIES}
)
