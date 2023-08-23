#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "Tools.h"
#include "ManipulatorNodeInterface.h"

namespace TaskIt {

// Forward declarations
class Object;

class PoseTracker {
    public:
        virtual bool update(Object& object) const = 0;

    protected:
        PoseTracker() {}
};

class SimulationPoseTracker : virtual public PoseTracker {
    public:
        SimulationPoseTracker(const ManipulatorNodeInterface& interface, const std::string& objects_ns = "objects");

        virtual bool update(Object& object) const override;

    protected:
        std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> m_planning_scene_interface;
};

class VRPNPoseTracker : virtual public PoseTracker {
    public:
        VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, double sampling_duration = 1.0);
        VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, const std::string& vrpn_client_topic, double sampling_duration = 1.0);

        virtual bool update(Object& object) const override;

    protected:
        bool getPose(Object& object, geometry_msgs::Pose& object_pose, double timeout = 5.0) const;

    private:
        double m_sampling_duration;
        std::shared_ptr<ros::NodeHandle> m_node_handle;
        std::string m_vrpn_client_topic;
};

class DynamicVRPNPoseTracker : public SimulationPoseTracker, public VRPNPoseTracker {
    public:
        DynamicVRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, const ManipulatorNodeInterface& interface, double sampling_duration = 1.0, const std::string& objects_ns = "objects");
        DynamicVRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, const ManipulatorNodeInterface& interface, const std::string& vrpn_client_topic, double sampling_duration = 1.0, const std::string& objects_ns = "objects");

        /// Set the threshold for the maximum translational distance the measured pose can be away from the simulated pose 
        /// before it considers the measurements faulty (and uses the simulaton_pose)
        /// @param distance Cartesian distance (m)
        void setTranslationThreshold(double distance) {m_translation_thresh = distance;}

        /// Set the threshold for the maximum rotation distance the measured pose can be away from the simulated pose 
        /// before it considers the measurements faulty (and uses the simulaton_pose)
        /// @param distance Shortest path angular separation (rad)
        void setRotationThreshold(double distance) {m_rotation_thresh = distance;}

        virtual bool update(Object& object) const override;
    
    private:
        bool isMeasuredPoseValid(const geometry_msgs::Pose& simulation_pose, const geometry_msgs::Pose& measured_pose) const;
    
    private:
        double m_translation_thresh = 0.3; // m
        double m_rotation_thresh = 0.09; // rad
};


}