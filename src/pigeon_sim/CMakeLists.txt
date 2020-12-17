cmake_minimum_required(VERSION 3.5.1)
project(pigeon_sim)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-Wall")
set(CMAKE_BUILD_TYPE Debug)


find_package(catkin REQUIRED 
  COMPONENTS 
  roscpp
  gazebo_ros)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(TinyXML2 8.0.0 MODULE REQUIRED)

find_package(ignition-math4 REQUIRED)

catkin_package(
)

# include_directories(${SDFormat_INCLUDE_DIRS})
# link_directories(${SDFormat_LIBRARY_DIRS})

add_executable(city_builder_node
  src/city_builder_node.cpp 
  src/city_builder.h
  src/gazebo_client.hpp
  src/model_primitives.hpp)

target_include_directories(
  city_builder_node 
  PUBLIC 
  ${TinyXML2_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

target_link_libraries(
  city_builder_node
  ignition-math4::ignition-math4
  ${TinyXML2_LIBRARIES}
  ${catkin_LIBRARIES})

install(TARGETS city_builder_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
