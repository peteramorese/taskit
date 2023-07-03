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
        virtual void update(Object& object) const = 0;

    protected:
        PoseTracker() {}
};

class SimulationPoseTracker : public PoseTracker {
    public:
        virtual void update(Object& object) const override {}
};

class VRPNPoseTracker : public PoseTracker {
    public:
        VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, double sampling_duration = 1.0);

        VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, const std::string& vrpn_client_topic, double sampling_duration = 1.0);

        virtual void update(Object& object) const override;
    private:
        //struct MsgContainer {
        //    tf2::Vector3 position;
        //    tf2::Quaternion orientation;

        //    MsgContainer() 
        //        : position(0.0f, 0.0f, 0.0f)
        //        , orientation(0.0f, 0.0f, 0.0f, 0.0f)
        //    {}

        //    uint32_t n_msgs = 0;
        //    auto onMsg = [&](const geometry_msgs::PoseStamped& msg) {
        //        tf2::Vector3 msg_position;
        //        tf2::Quaternion msg_orientation;
        //        tf2::fromMsg(msg.pose.position, msg_position);
        //        tf2::fromMsg(msg.pose.orientation, msg_orientation);
        //        position += msg_position;
        //        orientation += msg_orientation;
        //        ++n_msgs;
        //    };
        //};
    private:
        double m_sampling_duration;
        std::string m_vrpn_client_topic;
        std::shared_ptr<ros::NodeHandle> m_node_handle;
};

}