#include "Object.h"
#include "Tools.h"


namespace TaskIt {


void ObjectGroup::createObjects(const ros::NodeHandle& nh, const std::string& ns, const std::shared_ptr<PoseTracker>& pose_tracker) {

    std::vector<std::string> object_ids;
    nh.getParam(getParamName("object_ids", ns), object_ids);

    //std::vector<std::string> object_types;
    //nh.getParam(getParamName("object_types", ns), object_types);

    //std::vector<std::string> object_domains;
    //nh.param(getParamName("object_domains", ns), object_domains, {});

    //std::vector<std::string> object_orientation_types;
    //nh.getParam(getParamName("object_orientation_types", ns), object_orientation_types);

    //ROS_ASSERT_MSG(object_ids.size() == object_types.size(), "Each object name must correspond to a type");
    //ROS_ASSERT_MSG(object_ids.size() == object_orientation_types.size(), "Each object must have an orientation type");

    for (const auto& object_id : object_ids) {
        ROS_ASSERT_MSG(nh.hasParam(getParamName(object_id, ns)), "Missing configuration for at least one object");

        // Mandatory features
        ROS_ASSERT_MSG(nh.hasParam(getParamName(object_id, ns) + "/dimensions"), "Must specify 'dimensions' for each object");
        ROS_ASSERT_MSG(nh.hasParam(getParamName(object_id, ns) + "/primitive_type"), "Must specify 'primitive_type' for each object");
        
        // Make the object specification
        ObjectDimensionConfig dimension_cfg;
        nh.getParam(getParamName(object_id, ns) + "/dimensions", dimension_cfg);
        
        std::string primitive_type;
        nh.getParam(getParamName(object_id, ns) + "/primitive_type", primitive_type);

        ObjSpecPtr spec = ObjectSpecificationFactory::make(primitive_type, dimension_cfg);

        // Get orientation type if the config has it, otherwise set to up_x
        Quaternions::Type orientation_type = Quaternions::toType(nh.param<std::string>(getParamName(object_id + "/orientation_type", ns), "up_x"));

        std::string class_id = nh.param<>(getParamName(object_id, ns) + "/class", Object::s_default_class);
        Object object(object_id, spec, orientation_type, pose_tracker, class_id);

        if (nh.hasParam(getParamName(object_id, ns) + "/position")) {
            geometry_msgs::Pose pose;
            std::map<std::string, float> position_values;
            nh.getParam(getParamName(object_id, ns) + "/position", position_values);
            pose.position.x = position_values.at("x");
            pose.position.y = position_values.at("y");
            pose.position.z = position_values.at("z");
            pose.orientation.x = 0.0;
            pose.orientation.y = 0.0;
            pose.orientation.z = 0.0;
            pose.orientation.w = 1.0;
            ROS_INFO_STREAM("Found location for object '" << object_id << "' (x: " << position_values["x"] << " y: " << position_values["y"] << " z: " << position_values["z"] << ")");
            object.setPose(pose);
        }

        ROS_INFO_STREAM("Loaded object: " << object_id);
        insertObject(std::move(object));
    }

}

bool ObjectGroup::updatePlanningScene(moveit::planning_interface::PlanningSceneInterface& pci, const std::string& planning_frame_id, bool ignore_static, bool update_poses) {
    auto attached_objects = pci.getAttachedObjects();

    CollisionObjectVector collision_objects;
    collision_objects.reserve(size());

    for (auto& v_type : m_objects) {
        const auto& id = v_type.first;
        auto& obj = v_type.second;

        // Do not update if the object is attached
        auto it = attached_objects.find(id);
        if (it != attached_objects.end()) continue;

        // Do not update if the object is static
        if (ignore_static && obj.isStatic()) continue;

        if (update_poses && !obj.updatePose() && !obj.isStatic()) {
            ROS_WARN_STREAM("Failed to update pose for object ID '" << id << "'");
            return false;
        }

        moveit_msgs::CollisionObject col_obj = obj.getCollisionObject(planning_frame_id);

        col_obj.operation = col_obj.ADD;

        collision_objects.push_back(std::move(col_obj));
    }

    pci.applyCollisionObjects(collision_objects);

    return true;
}


}