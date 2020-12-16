cmake_minimum_required(VERSION 2.8.3)
project(acl_sim)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin)

catkin_package()



find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
)
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

if("$ENV{ROS_DISTRO}" STREQUAL "melodic")
	add_library(disable_physics src/disable_physics_melodic.cpp)
elseif("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
	add_library(disable_physics src/disable_physics_kinetic.cpp)
else()
	message(WARNING "Using the disable_physics kinetic version")
	add_library(disable_physics src/disable_physics_kinetic.cpp)
endif() 

target_link_libraries(disable_physics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_executable(create_random_forest src/create_random_forest.cpp)
target_link_libraries(create_random_forest ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${JPS3D_LIBRARIES} )

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
)

#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

#add_library(disable_physics SHARED ./src/disable_physics.cc)  
#target_link_libraries(disable_physics ${GAZEBO_LIBRARIES})

install(TARGETS create_random_forest
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/flightStatus.py
  scripts/monte_carlo_2.py
  scripts/perfect_tracker.py
  scripts/runSimForest.py
  scripts/modelRelay.py
  scripts/monte_carlo_3.py
  scripts/rosGazeboRelay.py
  scripts/spawn_field.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch meshes models urdf worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
