#pragma once

#include "ManipulatorNode.h"

#include "Object.h"
#include "Tools.h"

namespace ManipulationInterface {

template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_TYPES>
ManipulatorNode<OBJ_GROUP_T, ACTION_PRIMITIVES_TYPES...>::ManipulatorNode(const std::string& node_name, const std::string& planning_group, const std::string& frame_id, const std::shared_ptr<OBJ_GROUP_T>& obj_group, ACTION_PRIMITIVES_TYPES&&...action_primitives)
    : m_node_name(node_name)
    , m_action_primitives(std::forward<ACTION_PRIMITIVES_TYPES>(action_primitives)...)
    , m_obj_group(obj_group)
    , m_move_group(std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group))
    , m_planning_interface(std::make_shared<moveit::planning_interface::PlanningSceneInterface>())
    , m_visual_tools(std::make_shared<moveit_visual_tools::MoveItVisualTools>(frame_id))
    , m_frame_id(frame_id)
{
    DEBUG("Constructing...");

    //if constexpr (std::is_default_constructible_v<OBJ_GROUP_T>) ROS_ASSERT_MSG(obj_group, "Must provide an object group when using a non-default-constructable object group type");
    
    // Init ros items
    m_node_handle = std::make_unique<ros::NodeHandle>("~");
    m_spinner = std::make_unique<ros::AsyncSpinner>(2);
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
        m_planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                    "moveit_core", "planning_interface::PlannerManager"));
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

    // Set up action primitives
    m_action_services.reserve(sizeof...(ACTION_PRIMITIVES_TYPES));
    DEBUG("end of ctor");

}

template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_TYPES>
const std::shared_ptr<OBJ_GROUP_T> ManipulatorNode<OBJ_GROUP_T, ACTION_PRIMITIVES_TYPES...>::createWorkspace(const std::string& param_ns) {
    static_assert(std::is_default_constructible_v<OBJ_GROUP_T>, "Cannot createWorkspace with a non default constructable object group");

    m_obj_group.reset(new OBJ_GROUP_T);
    
    std::map<std::string, std::string> obj_domains;

    std::vector<std::string> obstacle_ids;
    m_node_handle->getParam(getParamName("obstacle_ids", param_ns), obstacle_ids);

    std::vector<std::string> obstacle_types;
    m_node_handle->getParam(getParamName("obstacle_types", param_ns), obstacle_types);

    std::vector<std::string> obstacle_domains;
    m_node_handle->param(getParamName("obstacle_domains", param_ns), obstacle_domains, {});

    std::vector<std::string> obstacle_orientation_types;
    m_node_handle->getParam(getParamName("obstacle_orientation_types", param_ns), obstacle_orientation_types);

    ROS_ASSERT_MSG(obstacle_ids.size() != obstacle_types.size(), "Each obstacle name must correspond to a type");
    ROS_ASSERT_MSG(obstacle_ids.size() != obstacle_orientation_types.size(), "Each obstacle must have an orientation type");

    m_collision_objects.reserve(obstacle_ids.size());

    for (int i=0; i<obstacle_ids.size(); ++i) {
        ObjectConfig config;
        m_node_handle->getParam(getParamName(obstacle_ids[i], param_ns), config);
        ROS_INFO_STREAM_NAMED(m_node_name, "Loaded obstacle: " << obstacle_ids[i] 
            << " at (x: " << config.at("x") 
            << ", y: " << config.at("y") 
            << ", z: " << config.at("z") 
            << ")");


        std::shared_ptr<ObjectSpecification> spec = makeObjectSpecification(obstacle_types[i], config);
        typename OBJ_GROUP_T::ObjectType object(obstacle_ids[i], spec, config, obstacle_orientation_types[i]);

        moveit_msgs::CollisionObject obstacle = object.getCollisionObject();
        obstacle.header.frame_id = m_frame_id;
        obstacle.operation = obstacle.ADD;

        if (i < obstacle_domains.size()) obj_domains[obstacle.id] = obstacle_domains[i];

        m_collision_objects.push_back(std::move(obstacle));

    }
    updateWorkspace();

    return m_obj_group;
}

template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_TYPES>
void ManipulatorNode<OBJ_GROUP_T, ACTION_PRIMITIVES_TYPES...>::updateWorkspace() {
    auto attached_objects = m_planning_interface->getAttachedObjects();

    for (auto& col_obj : m_collision_objects) {
        const auto& id = col_obj.id;

        auto it = attached_objects.find(id);
        if (it == attached_objects.end()) continue;

        auto obj_in_group = m_obj_group->getObject(id);
        obj_in_group.updatePose();
        col_obj = obj_in_group.getCollisionObject();

        col_obj.operation = col_obj.ADD;
    }

    m_planning_interface->applyCollisionObjects(m_collision_objects);

}


template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_TYPES>
std::string ManipulatorNode<OBJ_GROUP_T, ACTION_PRIMITIVES_TYPES...>::getParamName(const std::string& param_name, std::string ns) {
    if (ns.empty()) return "/" + param_name;
    if (ns.front() != '/') ns = "/" + ns;
    if (ns.back() != '/') ns.push_back('/');
    return ns + param_name;
}

}