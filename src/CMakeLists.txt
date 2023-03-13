add_executable(simple_manipulator_node simple_manipulator_node.cpp)
target_link_libraries(simple_manipulator_node 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    ManipulationInterface
)
target_include_directories(simple_manipulator_node PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${MANIPULATION_INTERFACE_LIBRARY_DIR}
)
install(TARGETS simple_manipulator_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(simple_manipulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})