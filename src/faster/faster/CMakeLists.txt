cmake_minimum_required(VERSION 2.8.3)
project(faster)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	snapstack_msgs
	faster_msgs
	sensor_msgs
	pcl_ros
	decomp_util
	decomp_ros_utils
	tf2_sensor_msgs
	behavior_selector
)
catkin_package()

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
#jps3d is not a catkin package
find_package(jps3d REQUIRED)
find_package(decomp_util REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")
find_package(GUROBI REQUIRED)


###########
## Build ##
###########

# setup global include directories

# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}")

add_executable(faster_node 
	src/main.cpp 
	src/faster.cpp 
	src/faster_ros.cpp 
	src/utils.cpp  
	src/jps_manager.cpp 
	src/solverGurobi.cpp)

target_include_directories( faster_node PUBLIC
	"include"
	${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
	${GUROBI_INCLUDE_DIRS}
	${JPS3D_INCLUDE_DIRS}
	${DECOMP_UTIL_INCLUDE_DIRS}
)

target_link_libraries(faster_node 
	${catkin_LIBRARIES} 
	${PCL_LIBRARIES} 
	${GUROBI_LIBRARIES}
	${JPS3D_LIBRARIES})

add_dependencies(faster_node ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS faster_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rqt_cfgs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY rviz_cfgs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY param DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/faster_commands.py scripts/goal_odom_to_cmd_vel_state.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})





