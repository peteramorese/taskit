#include "PoseTracker.h"

#include "Config.h"
#include "Object.h"
#include "PredicateHandler.h"

namespace TaskIt {

// Simulation Pose tracker
SimulationPoseTracker::SimulationPoseTracker(const ManipulatorNodeInterface& interface, const std::string& objects_ns)
    : m_planning_scene_interface(interface.planning_scene_interface.lock())
{}

bool SimulationPoseTracker::update(Object& object) const {
    // Get the pose from the planning scene interface
    std::map<std::string, geometry_msgs::Pose> objs = m_planning_scene_interface->getObjectPoses({object.id});

    auto it = objs.find(object.id);
    if (it != objs.end()) {
        object.setPose(it->second);
    } else {
        ROS_WARN("NOT FOUND");
    }
    return true;
}

// VRPN 
VRPNPoseTracker::VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, double sampling_duration)
    : m_node_handle(node_handle)
    , m_sampling_duration(sampling_duration)
{
    m_vrpn_client_topic = m_node_handle->param<std::string>(getParamName("vrpn_client_topic"), TASKIT_DEFAULT_VRPN_CLIENT_TOPIC);
}

VRPNPoseTracker::VRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, const std::string& vrpn_client_topic, double sampling_duration)
    : m_node_handle(node_handle)
    , m_vrpn_client_topic(vrpn_client_topic)
    , m_sampling_duration(sampling_duration)
{}

bool VRPNPoseTracker::update(Object& object) const {
    geometry_msgs::Pose object_pose;
    if (getPose(object, object_pose)) {
        object.setPose(object_pose);
        return true;
    }
    ROS_ERROR_STREAM("VRPN pose tracker publisher for object '" << object.id.c_str() << "' was not found");
    return false;
}

bool VRPNPoseTracker::getPose(Object& object, geometry_msgs::Pose& object_pose, double timeout) const {

    ROS_INFO_STREAM("Updating pose for object: " << object.id);
    std::string topic = m_vrpn_client_topic + object.id + "/pose";

    tf2::Vector3 position(0.0f, 0.0f, 0.0f);
    tf2::Quaternion orientation(0.0f, 0.0f, 0.0f, 0.0f);

    uint32_t n_msgs = 0;
    auto onMsg = [&](const geometry_msgs::PoseStamped::ConstPtr& msg) -> void {
        tf2::Vector3 msg_position;
        tf2::Quaternion msg_orientation;
        tf2::fromMsg(msg->pose.position, msg_position);
        tf2::fromMsg(msg->pose.orientation, msg_orientation);
        position += msg_position;
        orientation += msg_orientation;
        ++n_msgs;
    };

    // Create subscriber in scope so that the subscriber is destroyed after the duration
    ros::WallDuration duration(m_sampling_duration);
    uint16_t i = 0;
    while (true) {
        ros::Subscriber pose_sub = m_node_handle->subscribe<geometry_msgs::PoseStamped>(topic, 10, onMsg);
        duration.sleep();
        if (position.length() == 0.0f) {
            ROS_WARN_STREAM("VRPN Data for object '" << object.id.c_str() << "' may not be found on topic, retrying in " << timeout << " seconds (" << i << "/5 attempts)...");
            ros::WallDuration(timeout).sleep();
        } else {
            break;
        }
        if (i < 5) {
            ++i;
        } else {
            return false;
        }
    }

    position /= static_cast<float>(n_msgs);
    orientation /= static_cast<float>(n_msgs);
    orientation.normalize();

    tf2::toMsg(position, object_pose.position);
    tf2::convert(orientation, object_pose.orientation);
    return true;
}

// Dynamic VRPN 
DynamicVRPNPoseTracker::DynamicVRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, const ManipulatorNodeInterface& interface, double sampling_duration, const std::string& objects_ns) 
    : SimulationPoseTracker(interface, objects_ns)
    , VRPNPoseTracker(node_handle, sampling_duration)
{ }

DynamicVRPNPoseTracker::DynamicVRPNPoseTracker(const std::shared_ptr<ros::NodeHandle>& node_handle, const ManipulatorNodeInterface& interface, const std::string& vrpn_client_topic, double sampling_duration, const std::string& objects_ns) 
    : SimulationPoseTracker(interface, objects_ns)
    , VRPNPoseTracker(node_handle, vrpn_client_topic, sampling_duration)
{ }

bool DynamicVRPNPoseTracker::update(Object& object) const {

    // Get the simulation pose from the planning scene interface
    std::map<std::string, geometry_msgs::Pose> objs = m_planning_scene_interface->getObjectPoses({object.id});
    const geometry_msgs::Pose& simulation_pose = objs[object.id];

    // Get the measured pose from the VRPN data
    geometry_msgs::Pose measured_pose;
    if (getPose(object, measured_pose, 1.0) && isMeasuredPoseValid(simulation_pose, measured_pose)) {
        // If the object data was found, and the pose is near the simulation pose, set the objects pose to the measured pose
        object.setPose(measured_pose);
    } else {
        // Otherwise, either the object cannot be seen by the tracker, or the tracker is measuring poor data, so use the simulation pose
        ROS_WARN_STREAM("Data from VRPN pose publisher for object '" << object.id.c_str() << "' was either not found or faulty, updating using simulation pose");
        object.setPose(simulation_pose);
    }

    return true;
}

bool DynamicVRPNPoseTracker::isMeasuredPoseValid(const geometry_msgs::Pose& simulation_pose, const geometry_msgs::Pose& measured_pose) const {
    // Get tf2::Vector3
    tf2::Vector3 simulation_position, measured_position;
    tf2::fromMsg(simulation_pose.position, simulation_position);
    tf2::fromMsg(measured_pose.position, measured_position);

    if (simulation_position.distance(measured_position) > m_translation_thresh)
        return false;

    // Get tf2::Quaternion
    tf2::Quaternion simulation_orientation, measured_orientation;
    tf2::fromMsg(simulation_pose.orientation, simulation_orientation);
    tf2::fromMsg(measured_pose.orientation, measured_orientation);

    return simulation_orientation.angleShortestPath(measured_orientation) <= m_rotation_thresh;
}

}