#include "PredicateHandler.h"

namespace ManipulationInterface {

void PredicateHandler::createEnvironment(const ros::NodeHandle& nh, const std::string& environment_ns) {
    std::vector<std::string> location_names;
    nh.getParam(getParamName("location_names", environment_ns), location_names);

    std::vector<std::string> location_orientation_types;
    nh.getParam(getParamName("location_orientation_types", environment_ns), location_orientation_types);

    for (uint32_t i=0; i<location_names.size(); ++i) {
        std::map<std::string, float> location_properties;
        nh.getParam(getParamName(location_names[i], environment_ns), location_properties);
        ROS_INFO_STREAM("Loaded location: " << location_names[i] 
            << " at (x: " << location_properties.at("x") 
            << ", y: " << location_properties.at("y") 
            << ", z: " << location_properties.at("z") 
            << ", r: " << location_properties.at("r") 
            << ")");
        geometry_msgs::Point position;
        position.x = location_properties.at("x");
        position.y = location_properties.at("y");
        position.z = location_properties.at("z");
        addLocation(location_names[i], position, Quaternions::toType(location_orientation_types[i]), location_properties.at("r"));
    }
}

void PredicateHandler::setObjectPosesToLocations(const ros::NodeHandle& nh, const std::string& objects_ns) {
    std::map<std::string, std::string> initial_locations;
    nh.getParam(getParamName("initial_locations", objects_ns), initial_locations);

    for (const auto& v_type : initial_locations) {
        DEBUG("setting obj: " << v_type.first << " to location: " << v_type.second);
        DEBUG("pose x: " << getLocationPose(v_type.second).position.x);
        DEBUG("pose y: " << getLocationPose(v_type.second).position.y);
        DEBUG("pose z: " << getLocationPose(v_type.second).position.z);
        m_obj_group->getObject(v_type.first).pose = getLocationPose(v_type.second);
    }

}

double PredicateHandler::distance(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    tf2::Vector3 lhs_converted, rhs_converted; 
    tf2::fromMsg(lhs, lhs_converted);
    tf2::fromMsg(rhs, rhs_converted);
    return lhs_converted.distance(rhs_converted);
}

std::pair<bool, std::string> PredicateHandler::findPredicate(const geometry_msgs::Point& loc) const {
    double min_dist = -1.0;
    std::string nearest_predicate{};
    bool found = false;
    for (const auto& v_type : m_locations) {
        double d = distance(v_type.second.pose.position, loc);
        if (min_dist < 0.0 || d < min_dist) {
            min_dist = d;
            if (d <= v_type.second.detection_radius) {
                nearest_predicate = v_type.first;
                found = true;
            }
        }
    }
    DEBUG("found: " << found << " nearest predicate: " << nearest_predicate);
    return {found, nearest_predicate};
}

const PredicateHandler::PredicateSet PredicateHandler::getPredicates(const std::set<std::string>& ignore_obj_ids) const {
    PredicateSet predicate_set;
    for (const auto obj : m_obj_group->getObjects()) {
        DEBUG("b4 id lookup");
        const std::string& obj_id = obj->id;
        DEBUG("af id lookup");
        if (ignore_obj_ids.empty() || ignore_obj_ids.find(obj_id) != ignore_obj_ids.end()) {
            //const geometry_msgs::Pose& obj_pose = obj_loc.second;
            DEBUG("b4 find predicate");
            std::pair<bool, std::string> result = findPredicate(obj->pose.position);
            DEBUG("af find predicate");
            if (result.first) {
                DEBUG("b4 set obj predicate");
                predicate_set.setObjectPredicate(obj_id, result.second);
                DEBUG("af set obj predicate");
            } else {
                ROS_WARN_STREAM("Object with id: " << obj_id << " was not found within any predicate regions");
                predicate_set.success = false;
            }
        } else {
            predicate_set.setObjectPredicateIgnored(obj_id);
        }
    }
    DEBUG("returning predicate set");
    return predicate_set;
}


}