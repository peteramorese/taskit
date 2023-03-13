cmake_minimum_required(VERSION 3.12.1)
project(manipulation_interface)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-lstdc++fs)
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp 
	#vrpn_client_ros
	#interactive_markers
	moveit_core
	moveit_visual_tools
	moveit_ros_planning
	moveit_ros_planning_interface
	#moveit_ros_perception
	pluginlib
	geometric_shapes
	#pcl_ros
	#pcl_conversions
	#rosbag
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
	GraspSrv.srv
	ReleaseSrv.srv
	TransitSrv.srv
	#ActionSingle.srv
	#PlanningQuery.srv
	#PreferenceQuery.srv
	#RunQuery.srv
	#Strategy.srv
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
)

catkin_package(
	#  LIBRARIES vicon_franka_integration
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

#include_directories(
#	${catkin_INCLUDE_DIRS}
#	${Boost_INCLUDE_DIRS}
#	${EIGEN3_INCLUDE_DIRS}
#	)

set(MANIPULATION_INTERFACE_LIBRARY_DIR include)

add_subdirectory(library)
add_subdirectory(src)