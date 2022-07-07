cmake_minimum_required(VERSION 3.0.2)
project(manipulation_interface)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp 
	vrpn_client_ros
	interactive_markers
	moveit_core
	moveit_visual_tools
	moveit_ros_planning
	moveit_ros_planning_interface
	moveit_ros_perception
	pluginlib
	geometric_shapes
	pcl_ros
	pcl_conversions
	rosbag
	tf2_ros
	tf2_eigen
	tf2_geometry_msgs
	genmsg
	actionlib_msgs
	message_generation
	)

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

add_service_files(
	DIRECTORY srv
	FILES
	PlanningQuery.srv
	Strategy.srv
	)

generate_messages(
	DEPENDENCIES
	geometry_msgs
	std_msgs
	)

catkin_package(
	#  LIBRARIES vicon_franka_integration
	CATKIN_DEPENDS 
		vrpn_client_ros 
		geometry_msgs 
		tf2_ros
		tf2_geometry_msgs
		moveit_core
		moveit_visual_tools
		moveit_ros_planning_interface
		interactive_markers
		message_runtime
	DEPENDS
		EIGEN3
	#  DEPENDS system_lib
	)

include_directories(
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	)

add_subdirectory(task_planner)

add_executable(manipulator_node src/manipulator_node.cpp)
target_link_libraries(manipulator_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS manipulator_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(manipulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(manipulator_node_grasp src/manipulator_node_grasp.cpp)
target_link_libraries(manipulator_node_grasp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS manipulator_node_grasp DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(manipulator_node_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(com_node src/com_node.cpp)
target_link_libraries(com_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS com_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(com_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(planning_node src/planning_node.cpp)
target_link_libraries(planning_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS planning_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(manipulator_node_grasp src/manipulator_node_grasp.cpp)
#add_executable(com_node src/com_node.cpp)
#add_executable(planning_node src/planning_node.cpp)
#add_executable(andrew_node src/andrew_node.cpp)
#
#add_subdirectory(include)
#target_include_directories(planning_node PUBLIC include)
#link_directories(planning_node PUBLIC include)
#
#target_include_directories(andrew_node PUBLIC include)
#link_directories(andrew_node PUBLIC include)
#
#target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
#target_link_libraries(manipulator_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(manipulator_node_grasp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(com_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(planning_node 
#	${catkin_LIBRARIES} 
#	${Boost_LIBRARIES}
#	EdgeClass
#	AstarClass
#	StateSpaceClass
#	StateClass
#	ConditionClass
#	TransitionSystemClass
#	)
#target_link_libraries(andrew_node 
#	${catkin_LIBRARIES} 
#	${Boost_LIBRARIES}
#	EdgeClass
#	AstarClass
#	StateSpaceClass
#	StateClass
#	ConditionClass
#	TransitionSystemClass
#	)
#install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS manipulator_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS manipulator_node_grasp DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS com_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS planning_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS andrew_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(manipulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(manipulator_node_grasp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(com_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(planning_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(andrew_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
