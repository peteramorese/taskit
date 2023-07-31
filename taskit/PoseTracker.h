#pragma once

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Tools.h"
namespace TaskIt {

// Forward declarations
class Object;

class PoseTracker {
    public:
        virtual bool update(Object& object) const = 0;

    protected:
        PoseTracker() {}
};

class SimulationPoseTracker : public PoseTracker {
    public:
        virtual bool update(Object& object) const override {return true;}
};

class VRPNPoseTracker : public PoseTracker {
    public:
        VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, double sampling_duration = 1.0);

        VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, const std::string& vrpn_client_topic, double sampling_duration = 1.0);

        virtual bool update(Object& object) const override;

    private:
        double m_sampling_duration;
        std::string m_vrpn_client_topic;
        std::shared_ptr<ros::NodeHandle> m_node_handle;
};

}