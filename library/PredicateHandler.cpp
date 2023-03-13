#include "PredicateHandler.h"

namespace ManipulationInterface {

double PredicateHandler::distance(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    tf2::Vector3 lhs_converted, rhs_converted; 
    tf2::fromMsg(lhs, lhs_converted);
    tf2::fromMsg(rhs, rhs_converted);
    return lhs_converted.distance(rhs_converted);
}

std::pair<bool, std::string> PredicateHandler::findPredicate(const geometry_msgs::Point& loc) const {
    double min_dist = -1.0;
    std::string nearest_predicate = std::string();
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
    return {found, nearest_predicate};
}

const PredicateSet PredicateHandler::getPredicates(const std::vector<std::pair<std::string, geometry_msgs::Pose>>& obj_locs, std::set<std::string> ignore_obj_ids) const {
    PredicateSet predicate_set;
    for (const auto& obj_loc : obj_locs) {
        const std::string& obj_id = obj_loc.first;
        if (ignore_obj_ids.find(obj_id) != ignore_obj_ids.end()) {
            const geometry_msgs::Pose& obj_pose = obj_loc.second;
            auto result = findPredicate(obj_pose.position);
            if (result.first) {
                predicate_set.setObjectPredicate(obj_id, result.second);
            } else {
                ROS_WARN_STREAM("Object with id: " << obj_id << " was not found within any predicate regions");
                predicate_set.success = false;
            }
        } else {
            predicate_set.setObjectPredicateIgnored(obj_id);
        }
    }
    return predicate_set;
}

}