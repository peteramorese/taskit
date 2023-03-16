#pragma once

#include "ManipulatorNode.h"

#include "Object.h"
#include "Tools.h"

namespace ManipulationInterface {

template <class...ACTION_PRIMITIVES_TYPES>
ManipulatorNode<ACTION_PRIMITIVES_TYPES...>::ManipulatorNode(const std::string& node_name, const std::string& planning_group, const std::string& frame_id, ACTION_PRIMITIVES_TYPES&&...action_primitives)
    : m_node_name(node_name)
    , m_action_primitives(std::forward<ACTION_PRIMITIVES_TYPES>(action_primitives)...)
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
    
    // Read in the quaternion properties for the given arm
    Quaternions::readDefaultDownQuaternions(*m_node_handle, planning_group, "arm_config");

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
    DEBUG("end of ctor");

}

template <class...ACTION_PRIMITIVES_TYPES>
void ManipulatorNode<ACTION_PRIMITIVES_TYPES...>::createObjects(const std::string& workspace_ns, const std::shared_ptr<PoseTracker>& pose_tracker) {

    std::map<std::string, std::string> obj_domains;

    std::vector<std::string> object_ids;
    m_node_handle->getParam(getParamName("object_ids", workspace_ns), object_ids);

    std::vector<std::string> object_types;
    m_node_handle->getParam(getParamName("object_types", workspace_ns), object_types);

    std::vector<std::string> object_domains;
    m_node_handle->param(getParamName("object_domains", workspace_ns), object_domains, {});

    std::vector<std::string> object_orientation_types;
    m_node_handle->getParam(getParamName("object_orientation_types", workspace_ns), object_orientation_types);

    ROS_ASSERT_MSG(object_ids.size() != object_types.size(), "Each object name must correspond to a type");
    ROS_ASSERT_MSG(object_ids.size() != object_orientation_types.size(), "Each object must have an orientation type");

    m_collision_objects.reserve(object_ids.size());

    for (int i=0; i<object_ids.size(); ++i) {
        ObjectConfig config;
        m_node_handle->getParam(getParamName(object_ids[i], workspace_ns), config);
        ROS_INFO_STREAM_NAMED(m_node_name, "Loaded object: " << object_ids[i] 
            << " at (x: " << config.at("x") 
            << ", y: " << config.at("y") 
            << ", z: " << config.at("z") 
            << ")");


        std::shared_ptr<ObjectSpecification> spec = makeObjectSpecification(object_types[i], config);
        Object object(object_ids[i], spec, config, object_orientation_types[i], pose_tracker);

        moveit_msgs::CollisionObject collision_object = object.getCollisionObject();
        collision_object.header.frame_id = m_frame_id;
        collision_object.operation = collision_object.ADD;

        if (i < object_domains.size()) obj_domains[object.id] = object_domains[i];

        m_obj_group->insertObject(std::move(object));
        m_collision_objects.push_back(std::move(collision_object));

    }
    updateEnvironment();
}


template <class...ACTION_PRIMITIVES_TYPES>
void ManipulatorNode<ACTION_PRIMITIVES_TYPES...>::updateEnvironment() {
    auto attached_objects = m_planning_interface->getAttachedObjects();

    for (auto& col_obj : m_collision_objects) {
        const auto& id = col_obj.id;

        auto it = attached_objects.find(id);
        
        // Do not update if the object is attached
        if (it != attached_objects.end()) continue;

        auto& obj_in_group = m_obj_group->getObject(id);

        // Do not update if the object is static
        if (obj_in_group.isStatic()) continue;

        obj_in_group.updatePose();
        col_obj = obj_in_group.getCollisionObject();

        col_obj.operation = col_obj.ADD;
    }

    m_planning_interface->applyCollisionObjects(m_collision_objects);

}


}