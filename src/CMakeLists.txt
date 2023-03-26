# Basic Manipulator Node
add_executable(basic_manipulator_node_sim basic_manipulator_node_sim.cpp)
target_link_libraries(basic_manipulator_node_sim 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    ManipulationInterface
)
target_include_directories(basic_manipulator_node_sim PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${MANIPULATION_INTERFACE_LIBRARY_DIR}
)
install(TARGETS basic_manipulator_node_sim DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(basic_manipulator_node_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Advanced Manipulator Node
add_executable(advanced_manipulator_node_sim advanced_manipulator_node_sim.cpp)
target_link_libraries(advanced_manipulator_node_sim 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    ManipulationInterface
)
target_include_directories(advanced_manipulator_node_sim PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${MANIPULATION_INTERFACE_LIBRARY_DIR}
)
install(TARGETS advanced_manipulator_node_sim DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(advanced_manipulator_node_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})