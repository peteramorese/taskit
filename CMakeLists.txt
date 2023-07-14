cmake_minimum_required(VERSION 3.12.1)
project(taskit)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-lstdc++fs)
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp 
	#vrpn_client_ros
	moveit_core
	moveit_visual_tools
	moveit_ros_planning
	moveit_ros_planning_interface
	pluginlib
	geometric_shapes
	tf2_ros
	tf2_eigen
	tf2_geometry_msgs
	genmsg
	actionlib_msgs
	message_generation
)

#find_package(Franka REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(
	DIRECTORY srv
	FILES
	StowSrv.srv
	GraspSrv.srv
	ReleaseSrv.srv
	TransitSrv.srv
	UpdateEnvSrv.srv
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
)

catkin_package(
	CATKIN_DEPENDS 
		#vrpn_client_ros 
		geometry_msgs 
		tf2_ros
		tf2_geometry_msgs
		moveit_core
		moveit_visual_tools
		moveit_ros_planning_interface
		interactive_markers
		message_runtime
	DEPENDS
		#EIGEN3
	#  DEPENDS system_lib
)

set(TASKIT_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/taskit)

add_subdirectory(taskit)
add_subdirectory(src)
