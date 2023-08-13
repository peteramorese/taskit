#pragma once

#include "ManipulatorNode.h"

#include "visualization_msgs/MarkerArray.h"

#include "Object.h"
#include "Tools.h"
#include "ManipulatorProperties.h"

namespace TaskIt {

template <class...ACTION_PRIMITIVES_TYPES>
ManipulatorNode<ACTION_PRIMITIVES_TYPES...>::ManipulatorNode(const std::shared_ptr<ros::NodeHandle>& node_handle, const std::string& node_name, const std::string& planning_group, const std::string& planning_frame_id, ACTION_PRIMITIVES_TYPES&&...action_primitives)
    : m_node_name(node_name)
    , m_node_handle(node_handle)
    , m_action_primitives(std::forward<ACTION_PRIMITIVES_TYPES>(action_primitives)...)
    , m_move_group(std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group))
    , m_planning_scene_interface(std::make_shared<moveit::planning_interface::PlanningSceneInterface>())
    , m_state(std::make_shared<ManipulatorNodeState>())
    , m_auto_visualize(true)
{

    //if constexpr (std::is_default_constructible_v<OBJ_GROUP_T>) ROS_ASSERT_MSG(obj_group, "Must provide an object group when using a non-default-constructable object group type");
    
    // Init ros items
    m_spinner = std::make_shared<ros::AsyncSpinner>(2);
    m_spinner->start();

    // Init MoveIt items
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	m_robot_model = robot_model_loader.getModel();
    m_robot_state.reset(new robot_state::RobotState(m_robot_model));
    const robot_state::JointModelGroup* joint_model_group = m_robot_state->getJointModelGroup(planning_group);
    m_planning_scene = std::make_shared<planning_scene::PlanningScene>(m_robot_model);
    m_planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    // Load planning plugin
    std::string planner_plugin_name;

    if (!m_node_handle->getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try {
        m_planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    } catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try {
        m_planner_instance.reset(m_planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        //m_planner_instance = m_planner_plugin_loader->createInstance(planner_plugin_name);
        if (!m_planner_instance->initialize(m_robot_model, m_node_handle->getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planner '" << m_planner_instance->getDescription() << "'");
    } catch (pluginlib::PluginlibException& ex) {
        const std::vector<std::string>& classes = m_planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
    }

    setPlanner("RRTConnectkConfigDefault");

    // Setup the marker publisher
    m_visualizer.reset(new Visualizer(*m_node_handle, planning_frame_id, "/rviz_visual_tools"));

    // Set the pre action call back
    m_beforeActionCall = [this]() -> void {
        if (m_auto_visualize) m_visualizer->removeMarkers(Visualizer::MarkerType::Goal);
    };
}



template <class...ACTION_PRIMITIVES_TYPES>
bool ManipulatorNode<ACTION_PRIMITIVES_TYPES...>::updatePlanningScene(bool ignore_static, bool update_poses) {
    bool success = m_workspace_obj_group->updatePlanningScene(*m_planning_scene_interface, m_move_group->getPlanningFrame(), ignore_static, update_poses);
    success = success && m_obj_group->updatePlanningScene(*m_planning_scene_interface, m_move_group->getPlanningFrame(), ignore_static, update_poses);
    return success;
}


}