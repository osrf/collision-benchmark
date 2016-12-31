#################################################
# Find tinyxml. Only required to check for SDF validity.
find_package(tinyxml REQUIRED)
#message (STATUS "Found tinyxml: ${TINYXML_INCLUDE_DIRS}, ${TINYXML_LIBRARIES}")


#################################################
# Find Gazebo
find_package(gazebo 8.0 REQUIRED)


#################################################
# Set variables with all dependencies
set(dependencies_INCLUDE_DIRS
  ${TINYXML_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS})

set(dependencies_LIBRARY_DIRS
  ${TINYXML_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS})

set(dependencies_LIBRARIES
  ${TINYXML_LIBRARIES}
  ${GAZEBO_LIBRARIES})



