cmake_minimum_required(VERSION 2.8.3)
project(decomp_ros_utils)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS rviz roscpp decomp_ros_msgs)
# find_package(catkin_simple REQUIRED)
find_package(cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)

include_directories(include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${DECOMP_UTIL_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

## This setting causes Qt's "MOC" generation to happen automatically.
## this does not moc things in include!!!!!!! only in src
set(CMAKE_AUTOMOC ON)

## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
endif()

# catkin_simple()

set(SRC_FILES
  src/bound_visual.cpp
  src/mesh_visual.cpp
  src/vector_visual.cpp
  src/ellipsoid_array_visual.cpp
  src/ellipsoid_array_display.cpp
  src/polyhedron_array_display.cpp
  ${MOC_FILES})

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(decomp_rviz_plugins ${SRC_FILES})
target_link_libraries(decomp_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES} ${rviz_DEFAULT_PLUGIN_LIBRARIES})

install(TARGETS decomp_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES decomp_rviz_plugins
  CATKIN_DEPENDS
    rviz 
    roscpp 
    decomp_ros_msgs
)


# this is a hack, but file should have been put by package.xml <export> tag
# install(FILES "plugin_description.xml"
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#"share/${PROJECT_NAME}/cmake")
