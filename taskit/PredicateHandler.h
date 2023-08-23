#pragma once

#include <map>
#include <set>

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Quaternions.h"
#include "Object.h"

namespace TaskIt {

class PredicateHandler {
    public:
        static inline const std::string s_ignored = "ignored";
	public:
		class Location {
            public: 
                // Construct without a default class pose
                Location() = default;

                // Construct with a default class pose
                Location(const geometry_msgs::Pose& default_class_pose, float detection_radius_)
                    : m_detection_radius(detection_radius_)
                {
                    m_object_class_poses[Object::s_default_class] = default_class_pose;
                }

                // Construct with a default class pose
                Location(const geometry_msgs::Point& default_class_position, Quaternions::Type orientation_type, float detection_radius_)
                    : m_detection_radius(detection_radius_)
                {
                    geometry_msgs::Pose& default_pose = m_object_class_poses[Object::s_default_class];
                    default_pose.position = default_class_position;
                    default_pose.orientation = Quaternions::convert(Quaternions::get(orientation_type));
                }

                void addObjectClassPose(const std::string& object_class_id, const geometry_msgs::Pose& pose) {
                    if (!isClassPoseValid(pose.position)) {
                        ROS_ERROR_STREAM("Position corresponding to object class '" << object_class_id << "' is not within the detection radius (" << m_detection_radius << ") of the default-class position");
                    }
                    m_object_class_poses[object_class_id] = pose;
                }

                void addObjectClassPose(const std::string& object_class_id, const geometry_msgs::Point& position, Quaternions::Type orientation_type) {
                    if (!isClassPoseValid(position)) {
                        ROS_ERROR_STREAM("Position corresponding to object class '" << object_class_id << "' is not within the detection radius (" << m_detection_radius << ") of the default-class position");
                    }
                    geometry_msgs::Pose& pose = m_object_class_poses[object_class_id];
                    pose.position = position;
                    pose.orientation = Quaternions::convert(Quaternions::get(orientation_type));
                }

                const geometry_msgs::Pose& defaultPose() const {return m_object_class_poses.at(Object::s_default_class);}
                const geometry_msgs::Pose& getPose(const std::string& object_class_id) const {
                    auto it = m_object_class_poses.find(object_class_id);
                    if (it != m_object_class_poses.end()) {
                        return it->second;
                    } else {
                        ROS_WARN_STREAM("Did not find a specific pose for object class '" << object_class_id << "', using 'default'");
                        return defaultPose();
                    }
                }
                double& detectionRadius() {return m_detection_radius;}
                double detectionRadius() const {return m_detection_radius;}

            private:
                bool isClassPoseValid(const geometry_msgs::Point& position) const {
                    auto it = m_object_class_poses.find(Object::s_default_class);
                    if (it != m_object_class_poses.end()) {
                        tf2::Vector3 default_class_position, spec_class_position;
                        tf2::fromMsg(it->second.position, default_class_position);
                        tf2::fromMsg(position, spec_class_position);
                        return tf2::tf2Distance(default_class_position, spec_class_position) <= m_detection_radius;
                    }
                    return true;
                }

            private:
                std::map<std::string, geometry_msgs::Pose> m_object_class_poses;
                double m_detection_radius = 0.05;

		};
        struct PredicateSet {
            public:
                std::pair<bool, std::string> lookupObjectPredicate(const std::string& obj_id) const {
                    auto it = m_obj_id_to_location_name.find(obj_id);
                    return (it != m_obj_id_to_location_name.end()) ? std::make_pair(true, it->second) : std::make_pair(false, std::string());
                }

                std::pair<bool, std::string> lookupLocationPredicate(const std::string& location_name) const {
                    std::pair<bool, std::string> ret = std::make_pair(false, std::string());
                    for (const auto v_type : m_obj_id_to_location_name) {
                        if (v_type.second == location_name && !ret.first) 
                            ret = std::make_pair(true, v_type.first);
                        else if (v_type.second == location_name)
                            ROS_WARN_STREAM("Found multiple objects in location '" << location_name << "'");
                    }
                    return ret;
                }

                void setObjectPredicate(const std::string& obj_id, const std::string& location_name) {
                    m_obj_id_to_location_name.insert(std::make_pair(obj_id, location_name));
                }

                void setObjectPredicateIgnored(const std::string& obj_id) {
                    m_obj_id_to_location_name.insert(std::make_pair(obj_id, s_ignored));
                }

                bool assertAllObjectsFound(const ObjectGroup& obj_group) const {
                    for (const auto& id : obj_group.getIds()) {
                        if (m_obj_id_to_location_name.find(id) == m_obj_id_to_location_name.end()) return false;
                    }
                    return true;
                }

                bool success = true;

            private:
                std::map<std::string, std::string> m_obj_id_to_location_name;
        };
	public:
        //std::size_t size() const {return m_locations.size(); }
        PredicateHandler(const std::shared_ptr<ObjectGroup>& obj_group) : m_obj_group(obj_group) {}

        void createEnvironment(const ros::NodeHandle& nh, const std::string& environment_ns);

        const std::map<std::string, Location>& getLocations() const {return m_locations;}

        // Set the poses for each object to the discrete location defined under `initial_locations` param. Does nothing
        // if `initial_locations` is not found
        void setObjectPosesToLocations(const ros::NodeHandle& nh, const std::string& objects_ns = "objects");

        void addLocation(const std::string& name, Location&& location) {
            m_locations.insert(std::make_pair(name, std::move(location)));
        }

        const geometry_msgs::Pose& getLocationPose(const std::string& name, const std::string& object_class = "default") const {
            return m_locations.at(name).getPose(object_class);
        }

        // Look up
		const PredicateSet getPredicates(const std::set<std::string>& ignore_obj_ids = std::set<std::string>{}) const;
    private:
		static double distance(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs);

        std::pair<bool, std::string> findPredicate(const Object* obj) const;

    private:
        std::map<std::string, Location> m_locations;
        std::shared_ptr<ObjectGroup> m_obj_group; 
};
}