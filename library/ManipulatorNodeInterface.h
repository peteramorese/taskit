#pragma once

// STL
#include <memory>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "Gripper.h"

namespace ManipulationInterface {

// Forward declarations
class ObjectGroup;
class PredicateHandler;
class ManipulatorNodeVisualizer;
class ManipulatorNodeVisualizer;

struct ManipulatorNodeInterface {
    std::weak_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    std::weak_ptr<moveit::planning_interface::PlanningSceneInterface> planning_interface;
    std::weak_ptr<ObjectGroup> object_group;
    std::weak_ptr<PredicateHandler> predicate_handler;
    std::weak_ptr<ManipulatorNodeVisualizer> visualizer;

    ManipulatorNodeInterface(
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_, 
        const std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_interface_,
        const std::shared_ptr<ObjectGroup>& object_group_,
        const std::shared_ptr<PredicateHandler>& predicate_handler_,
        const std::shared_ptr<ManipulatorNodeVisualizer>& visualizer_)
        : move_group(move_group_)
        , planning_interface(planning_interface_)
        , object_group(object_group_)
        , predicate_handler(predicate_handler_)
        , visualizer(visualizer_)
    {}


};

struct ConstManipulatorNodeInterface {
    std::weak_ptr<const moveit::planning_interface::MoveGroupInterface> move_group;
    std::weak_ptr<const moveit::planning_interface::PlanningSceneInterface> planning_interface;
    std::weak_ptr<const ObjectGroup> object_group;
    std::weak_ptr<const PredicateHandler> predicate_handler;
    std::weak_ptr<const ManipulatorNodeVisualizer> visualizer;

    ConstManipulatorNodeInterface(
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_, 
        const std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_interface_,
        const std::shared_ptr<ObjectGroup>& object_group_,
        const std::shared_ptr<PredicateHandler>& predicate_handler_,
        const std::shared_ptr<ManipulatorNodeVisualizer>& visualizer_)
        : move_group(move_group_)
        , planning_interface(planning_interface_)
        , object_group(object_group_)
        , predicate_handler(predicate_handler_)
        , visualizer(visualizer_)
    {}
};


}