# Manipulator Node Simulation
add_executable(manipulator_node_sim manipulator_node_sim.cpp)
target_link_libraries(manipulator_node_sim 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    TaskIt
)
target_include_directories(manipulator_node_sim PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${TASKIT_LIBRARY_DIR}
)
install(TARGETS manipulator_node_sim DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(manipulator_node_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Manipulator Node Real
add_executable(manipulator_node_real manipulator_node_real.cpp)
target_link_libraries(manipulator_node_real 
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
    TaskIt
)
target_include_directories(manipulator_node_real PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${TASKIT_LIBRARY_DIR}
)
install(TARGETS manipulator_node_real DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(manipulator_node_real ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
