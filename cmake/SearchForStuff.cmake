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
find_package(gazebo 9.0 REQUIRED)

find_package(assimp REQUIRED)

#################################################
# Set variables with all dependencies
set(dependencies_INCLUDE_DIRS
  ${assimp_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${QT_INCLUDE_DIR})

set(dependencies_LIBRARY_DIRS
  ${assimp_LIBRARY_DIRS}
  ${TINYXML_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS})

set(dependencies_LIBRARIES
  ${assimp_LIBRARIES}
  ${TINYXML_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${Qt5Core_LIBRARIES}
  ${Qt5Widgets_LIBRARIES})
