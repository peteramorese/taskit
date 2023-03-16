#pragma once

#include <map>
#include <set>

#include "geometry_msgs/Pose.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Quaternions.h"
#include "Object.h"

namespace ManipulationInterface {

class PredicateHandler {
    public:
        static inline const std::string s_ignored = "ignored";
	public:
		struct Location {
            Location(const geometry_msgs::Pose& pose_, float detection_radius_)
                : pose(pose_)
                , detection_radius(detection_radius_)
            {}

			geometry_msgs::Pose pose;
            float detection_radius;
		};
        struct PredicateSet {
            public:
                std::pair<bool, const std::string&> lookupObjectPredicate(const std::string& obj_id) const {
                    auto it = m_obj_id_to_location_name.find(obj_id);
                    return (it != m_obj_id_to_location_name.end()) ? std::make_pair(true, it->second) : std::make_pair(false, std::string());
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
                std::map<std::string, const std::string&> m_obj_id_to_location_name;
        };
	public:
        //std::size_t size() const {return m_locations.size(); }

		void addLocation(const std::string& name, const geometry_msgs::Point& position, Quaternions::Type orientation_type, float detection_radius) {
            geometry_msgs::Pose pose;
            pose.position = position;
            pose.orientation = Quaternions::convert(Quaternions::get(orientation_type));
            m_locations.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(pose, detection_radius));
		}

		void addLocation(const std::string& name, const geometry_msgs::Pose& pose, float detection_radius) {
            m_locations.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(pose, detection_radius));
		}

        void addLocation(const std::string& name, const Location& location) {
            m_locations.insert(std::make_pair(name, location));
        }

        const geometry_msgs::Pose& getLocationPose(const std::string& name) const {return m_locations.at(name).pose;}

		const PredicateSet getPredicates(const std::vector<std::pair<std::string, geometry_msgs::Pose>>& obj_locs, std::set<std::string> ignore_obj_ids) const;
    private:
		static double distance(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs);

        std::pair<bool, std::string> findPredicate(const geometry_msgs::Point& loc) const;

    private:
        std::map<std::string, Location> m_locations;
};
}