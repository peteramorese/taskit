#include "Object.h"
#include "Tools.h"


namespace ManipulationInterface {


void ObjectGroup::createObjects(const ros::NodeHandle& nh, const std::string& ns, const std::string& frame_id, CollisionObjectVector& collision_objs, const std::shared_ptr<PoseTracker>& pose_tracker) {

    DEBUG("creating objects");
    // TODO

    std::vector<std::string> object_ids;
    nh.getParam(getParamName("object_ids", ns), object_ids);

    std::vector<std::string> object_types;
    nh.getParam(getParamName("object_types", ns), object_types);

    std::vector<std::string> object_domains;
    nh.param(getParamName("object_domains", ns), object_domains, {});

    std::vector<std::string> object_orientation_types;
    nh.getParam(getParamName("object_orientation_types", ns), object_orientation_types);

    ROS_ASSERT_MSG(object_ids.size() != object_types.size(), "Each object name must correspond to a type");
    ROS_ASSERT_MSG(object_ids.size() != object_orientation_types.size(), "Each object must have an orientation type");

    collision_objs.reserve(collision_objs.size() + object_ids.size());

    for (uint32_t i=0; i<object_ids.size(); ++i) {
        ObjectConfig config;
        nh.getParam(getParamName(object_ids[i], ns), config);

        ROS_INFO_STREAM("Loaded object: " << object_ids[i]);

        std::shared_ptr<ObjectSpecification> spec = makeObjectSpecification(object_types[i], config);
        Object object(object_ids[i], spec, config, object_orientation_types[i], pose_tracker);

        moveit_msgs::CollisionObject collision_object = object.getCollisionObject();
        collision_object.header.frame_id = frame_id;
        collision_object.operation = collision_object.ADD;

        //if (i < object_domains.size()) obj_domains[object.id] = object_domains[i];

        insertObject(std::move(object));
        collision_objs.push_back(std::move(collision_object));

    }
}


}