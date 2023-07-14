add_library(TaskIt 
    PredicateHandler.cpp
    Object.cpp
    PoseTracker.cpp
    Visualizer.cpp
)
set_property(TARGET TaskIt PROPERTY CXX_STANDARD 14)
target_link_libraries(TaskIt PUBLIC
    ${catkin_LIBRARIES} 
    ${Boost_LIBRARIES}
)
target_include_directories(TaskIt PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${MANIPULATION_INTERFACE_LIBRARY_DIR}
)
install(TARGETS TaskIt DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
add_dependencies(TaskIt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})