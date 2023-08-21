#include "PredicateHandler.h"

namespace TaskIt {

void PredicateHandler::createEnvironment(const ros::NodeHandle& nh, const std::string& environment_ns) {
    std::vector<std::string> location_names;
    nh.getParam(getParamName("location_names", environment_ns), location_names);

    for (const auto& location_name : location_names) {
        ROS_ASSERT_MSG(nh.hasParam(getParamName(location_name, environment_ns)), "Missing configuration for at least one location");

        Location location;

        // Detection radius
        ROS_ASSERT_MSG(nh.hasParam(getParamName(location_name + "/detection_radius", environment_ns)), "Must specify a 'detection_radius' for each location");
        nh.getParam(getParamName(location_name + "/detection_radius", environment_ns), location.detectionRadius());

        // Orientation type
        if (!nh.hasParam(getParamName(location_name + "/orientation_type", environment_ns))) {
            ROS_WARN_STREAM("Did not find orientation type for location '" << location_name << "', setting to up_x");
        }
        Quaternions::Type orientation_type = Quaternions::toType(nh.param<std::string>(getParamName(location_name + "/orientation_type", environment_ns), "up_x"));
        
        // Default object class location
        ROS_ASSERT_MSG(nh.hasParam(getParamName(location_name + "/default" , environment_ns)) 
            || nh.hasParam(getParamName(location_name + "/object_classes", environment_ns)), "Must specify either a 'default' object class location and/or 'object_classes'");

        if (nh.hasParam(getParamName(location_name + "/default", environment_ns))) {
            std::map<std::string, float> location_properties;
            nh.getParam(getParamName(location_name + "/default", environment_ns), location_properties);

            ROS_INFO_STREAM("Loaded default object class location: " << location_name 
                << " at (x: " << location_properties.at("x") 
                << ", y: " << location_properties.at("y") 
                << ", z: " << location_properties.at("z") 
                << ")");
            geometry_msgs::Point position;
            position.x = location_properties.at("x");
            position.y = location_properties.at("y");
            position.z = location_properties.at("z");
            location.addObjectClassPose(Object::s_default_class, position, orientation_type);
        } else {
            ROS_WARN_STREAM("No default class location was specified for location '" << location_name << "'. Detection radius checks will not be performed");
        }

        // Get list of specific object classes
        std::vector<std::string> object_classes;
        nh.getParam(getParamName(location_name, environment_ns) + "/object_classes", object_classes);

        // Insert pose for each object class
        for (const auto& object_class : object_classes) {
            ROS_ASSERT_MSG(nh.hasParam(getParamName(location_name + "/" + object_class, environment_ns)), "Must specify a coordinate for each object class");
            std::map<std::string, float> location_properties;
            nh.getParam(getParamName(location_name + "/" + object_class, environment_ns), location_properties);

            ROS_INFO_STREAM("Loaded '" << object_class << "' object class location: " << location_name 
                << " at (x: " << location_properties.at("x") 
                << ", y: " << location_properties.at("y") 
                << ", z: " << location_properties.at("z") 
                << ")");
            geometry_msgs::Point position;
            position.x = location_properties.at("x");
            position.y = location_properties.at("y");
            position.z = location_properties.at("z");
            location.addObjectClassPose(object_class, position, orientation_type);
        }

        addLocation(location_name, std::move(location));
    }
}

void PredicateHandler::setObjectPosesToLocations(const ros::NodeHandle& nh, const std::string& objects_ns) {
    for (Object* obj : m_obj_group->getObjects()) {
        std::string initial_location;
        if (nh.getParam(getParamName(obj->id, objects_ns) + "/initial_location", initial_location)) {
            ROS_ASSERT_MSG(m_locations.find(initial_location) != m_locations.end(), "Unrecognized object initial location");
            obj->setPose(getLocationPose(initial_location, obj->class_id));
        }

    }
}

double PredicateHandler::distance(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    tf2::Vector3 lhs_converted, rhs_converted; 
    tf2::fromMsg(lhs, lhs_converted);
    tf2::fromMsg(rhs, rhs_converted);
    return lhs_converted.distance(rhs_converted);
}

std::pair<bool, std::string> PredicateHandler::findPredicate(const Object* obj) const {
    double min_dist = -1.0;
    std::string nearest_predicate{};
    bool found = false;
    for (const auto& kv : m_locations) {
        const geometry_msgs::Point& location_position = kv.second.getPose(obj->class_id).position;
        double d = distance(obj->pose().position, location_position);
        if (min_dist < 0.0 || d < min_dist) {
            min_dist = d;
            if (d <= kv.second.detectionRadius()) {
                nearest_predicate = kv.first;
                found = true;
            }
        }
    }
    return {found, nearest_predicate};
}

const PredicateHandler::PredicateSet PredicateHandler::getPredicates(const std::set<std::string>& ignore_obj_ids) const {
    PredicateSet predicate_set;
    for (const auto obj : m_obj_group->getObjects()) {
        const std::string& obj_id = obj->id;
        if (ignore_obj_ids.empty() || ignore_obj_ids.find(obj_id) == ignore_obj_ids.end()) {
            std::pair<bool, std::string> result = findPredicate(obj);
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