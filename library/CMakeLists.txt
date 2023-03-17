add_library(ManipulationInterface 
    PredicateHandler.cpp
    Object.cpp
)
target_link_libraries(ManipulationInterface PUBLIC
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
)
target_include_directories(ManipulationInterface PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${MANIPULATION_INTERFACE_LIBRARY_DIR}
)
install(TARGETS ManipulationInterface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(ManipulationInterface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
