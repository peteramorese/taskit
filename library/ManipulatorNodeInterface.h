#pragma once

// STL
#include <memory>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
//#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace ManipulationInterface {

// Forward declarations
class ObjectGroup;

struct ManipulatorNodeInterface {
    std::weak_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    std::weak_ptr<moveit::planning_interface::PlanningSceneInterface> planning_interface;
    std::weak_ptr<ObjectGroup> object_group;

    ManipulatorNodeInterface(
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_, 
        const std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_interface_,
        const std::shared_ptr<ObjectGroup>& object_group_)
        : move_group(move_group_)
        , planning_interface(planning_interface_)
        , object_group(object_group_)
    {}
};

struct ConstManipulatorNodeInterface {
    std::weak_ptr<const moveit::planning_interface::MoveGroupInterface> move_group;
    std::weak_ptr<const moveit::planning_interface::PlanningSceneInterface> planning_interface;
    std::weak_ptr<const ObjectGroup> object_group;

    ConstManipulatorNodeInterface(
        const std::shared_ptr<moveit::planning_interface::MoveGroupInterface>& move_group_, 
        const std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>& planning_interface_,
        const std::shared_ptr<ObjectGroup>& object_group_)
        : move_group(move_group_)
        , planning_interface(planning_interface_)
        , object_group(object_group_)
    {}
};


}