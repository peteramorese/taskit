#pragma once

#include "ManipulatorNode.h"

#include "Object.h"

namespace ManipulationInterface {

template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_T>
ManipulatorNode<ACTION_PRIMITIVES_T...>::ManipulatorNode(int argc, char** argv, const std::string& planning_group, const std::string& frame_id, const std::shared_ptr<OBJ_GROUP_T>& obj_group, ACTION_PRIMITIVES_T&&...action_primitives)
    : m_action_primitives(std::forward<ACTION_PRIMITIVES_T>(action_primitives)...)
    , m_obj_group(obj_group)
    , m_move_group(std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group))
    , m_planning_interface(std::make_shared<moveit::planning_interface::PlanningSceneInterface>())
    , m_visual_tools(std::make_shared<>(frame_id))
    , m_frame_id(frame_id)
{
    // Init ros items
    ros::init(argc, argv, s_node_name);
    m_node_handle.reset(std::make_unique<ros::NodeHandle>("~"));
    m_spinner.reset(std::make_unique<ros::AsyncSpinner>(2));
    m_spinner->start();

    // Init MoveIt items
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model_loader.getModel()));
    const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group);
    m_planning_scene(new planning_scene::PlanningScene(robot_model_loader.getModel()));
    m_planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
    m_visual_tools(m_frame_id);

    // Load planning plugin
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    if (!M_NH.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
                    "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex) {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, M_NH.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planner '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException& ex) {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl << "Available plugins: " << ss.str());
    }

    setPlanner("RRTConnectkConfigDefault");

    // Set up action primitives
    m_action_services.reserve(sizeof...(ACTION_PRIMITIVES_T));

}

template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_T>
bool ManipulatorNode<ACTION_PRIMITIVES_T...>::createWorkspace(const std::string& param_ns) {

    std::map<std::string, std::string> obj_domains;

    std::vector<std::string> obstacle_ids;
    M_NH.getParam(getParamName("obstacle_ids", param_ns), obstacle_ids);

    std::vector<std::string> obstacle_types;
    M_NH.getParam(getParamName("obstacle_types", param_ns), obstacle_types);

    std::vector<std::string> obstacle_domains;
    M_NH.getParam(getParamName("obstacle_domains", param_ns), obstacle_domains, {});

    std::vector<std::string> obstacle_orientation_types;
    M_NH.getParam(getParamName("obstacle_orientation_types", param_ns), obstacle_orientation_types);

    ROS_ASSERT_MSG(obstacle_ids.size() != obstacle_types.size(), "Each obstacle name must correspond to a type");
    ROS_ASSERT_MSG(obstacle_ids.size() != obstacle_orientation_types.size(), "Each obstacle must have an orientation type");

    m_collision_objects.reserve(obstacle_ids.size());

    for (int i=0; i<obstacle_ids.size(); ++i) {
        ObjectConfig config;
        M_NH.getParam(getParamName(param_ns, obstacle_ids[i]), config);
        ROS_INFO_STREAM_NAMED(s_node_name, "Loaded obstacle: " << config 
            << " at (x: " << config.at("x") 
            << ", y: " << config.at("y") 
            << ", z: " << config.at("z") 
            << ")");


        std::shared_ptr<ObjectSpecification> spec = makeObjectSpecification(obstacle_types[i], config);
        OBJ_GROUP_T::ObjectType object(obstacle_ids[i], spec, config, obstacle_orientation_types[i]);

        moveit_msgs::CollisionObject obstacle = object.getCollisionObject();
        obstacle.operation = obstacle.ADD;

        if (i < obstacle_domains.size()) obj_domains[obstacle.id] = obstacle_domains[i];

        m_collision_objects.push_back(std::move(obstacle));

    }

}

template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_T>
void ManipulatorNode<ACTION_PRIMITIVES_T...>::updateWorkspace() {
    auto attached_objects = m_planning_scene->getAttachedObjects();

    for (auto& col_obj : m_collision_objects) {
        const auto& id = col_obj.id;

        auto it = attached_objects.find(id);
        if (it == attached_objects.end()) continue;

        auto obj_in_group = m_obj_group.getObject(id);
        obj_in_group.updatePose();
        col_obj = obj_in_group.getCollisionObject();

        col_obj.operation = col_obj.ADD;
    }

    m_planning_scene->applyCollisionObjects(m_collision_objects);

}


template <class OBJ_GROUP_T, class...ACTION_PRIMITIVES_T>
std::string ManipulatorNode<ACTION_PRIMITIVES_T...>::getParamName(const std::string& param_name, std::string namespace = std::string()) {
    if (namespace.empty) return "/" + param_name;
    if (namespace.front() != '/') namespace = "/" + namespace;
    if (namespace.back() != '/') namespace.push_back('/');
    return namespace + param_name;
}

}