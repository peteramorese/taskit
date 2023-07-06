#include "Object.h"
#include "Tools.h"


namespace TaskIt {


void ObjectGroup::createObjects(const ros::NodeHandle& nh, const std::string& ns, const std::shared_ptr<PoseTracker>& pose_tracker) {

    std::vector<std::string> object_ids;
    nh.getParam(getParamName("object_ids", ns), object_ids);

    std::vector<std::string> object_types;
    nh.getParam(getParamName("object_types", ns), object_types);

    std::vector<std::string> object_domains;
    nh.param(getParamName("object_domains", ns), object_domains, {});

    std::vector<std::string> object_orientation_types;
    nh.getParam(getParamName("object_orientation_types", ns), object_orientation_types);

    ROS_ASSERT_MSG(object_ids.size() == object_types.size(), "Each object name must correspond to a type");
    ROS_ASSERT_MSG(object_ids.size() == object_orientation_types.size(), "Each object must have an orientation type");

    for (uint32_t i=0; i<object_ids.size(); ++i) {
        ObjectConfig config;
        nh.getParam(getParamName(object_ids[i], ns), config);

        ROS_INFO_STREAM("Loaded object: " << object_ids[i]);

        std::shared_ptr<ObjectSpecification> spec = makeObjectSpecification(object_types[i], config);
        Object object(object_ids[i], spec, config, Quaternions::toType(object_orientation_types[i]), pose_tracker);

        insertObject(std::move(object));
    }

}

void ObjectGroup::updatePosesWithPlanningScene(moveit::planning_interface::PlanningSceneInterface& pci, const std::string& planning_frame_id, bool ignore_static) {
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

        obj.updatePose();
        moveit_msgs::CollisionObject col_obj = obj.getCollisionObject(planning_frame_id);

        col_obj.operation = col_obj.ADD;

        collision_objects.push_back(std::move(col_obj));
    }

    pci.applyCollisionObjects(collision_objects);
}


}