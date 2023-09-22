cmake_minimum_required(VERSION 3.12.1)
project(taskit)
set(CMAKE_CXX_STANDARD 17)
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

add_message_files(
	DIRECTORY msg
	FILES
	MovementProperties.msg
)

add_service_files(
	DIRECTORY srv
	FILES
	GetObjectLocations.srv
	Stow.srv
	Grasp.srv
	Release.srv
	Transit.srv
	UpdateEnv.srv
)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
	#taskit
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

# CMakeLists.txt should install all the scripts explictly using the special install
# function catkin_install_python. This will make sure that shebang lines are updated to use
# the specific Python version used at configure time.
catkin_install_python(PROGRAMS 
		icra24_exp_scripts/ttt_robotwin_no_stochasticity.py
 		DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

set(TASKIT_LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/taskit)

add_definitions(-DROS_ASSERT_ENABLED)

add_subdirectory(taskit)
add_subdirectory(src)
