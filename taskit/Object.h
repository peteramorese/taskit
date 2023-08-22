#pragma once

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "Quaternions.h"
#include "PoseTracker.h"
#include "Tools.h"


namespace TaskIt {

using ObjectDimensionConfig = std::map<std::string, float>;
static double getDimensionConfigValue(const ObjectDimensionConfig& cfg, const std::string& key) {
    auto it = cfg.find(key);
    if (it != cfg.end()) {
        return it->second;
    }
    ////
    //for (auto kv : cfg) {
    //    DEBUG("key: " << kv.first);
    //}
    ////
    ROS_ERROR_STREAM("Object dimension configuration is missing value for '" << key << "'");
    ROS_ASSERT(false);
    return 0.0;
}

// Object specifications

enum class ObjectPrimitive {
    None,
    Box,
    Sphere,
    Cylinder
};

static ObjectPrimitive getObjectPrimitiveType(const std::string& str) {
    if (str == "box") return ObjectPrimitive::Box;
    if (str == "sphere") return ObjectPrimitive::Sphere;
    if (str == "cylinder") return ObjectPrimitive::Cylinder;
    ROS_ERROR_STREAM("Unrecognized primitive type: " << str);
    return ObjectPrimitive::None;
}

struct GripperSpecification {
    // Grip properties
    double grip_force = 50.0;
    double grip_speed = 0.1;
    double grip_width_closed = 0.05;
    double grip_epsilon_inner = 0.03;
    double grip_epsilon_outter = 0.03;
};

struct ObjectSpecification : public GripperSpecification {
    public: 
        double placement_safety_distance = 0.01;

    public:
        virtual moveit_msgs::CollisionObject getCollisionObject(const geometry_msgs::Pose& pose) const = 0;
        virtual float getLengthOffset() const = 0;
        virtual float getWidthOffset() const = 0;
        virtual float getHeightOffset() const = 0;
    
    protected:
        virtual void constructFromConfig(const ObjectDimensionConfig& config) = 0;
};

using ObjSpecPtr = std::shared_ptr<ObjectSpecification>;

class ObjectSpecificationFactory {
    public:
        static void hello(const std::string& test) {}

        template <class SPEC_T>
        static void registerType(const std::string& spec_type_id) {
            m_creators[spec_type_id] = [](const ObjectDimensionConfig& cfg) -> ObjSpecPtr {return std::make_shared<SPEC_T>(cfg);};
        }

        static ObjSpecPtr make(const std::string& spec_type_id, const ObjectDimensionConfig& config) {
            auto it = m_creators.find(spec_type_id);
            if (it != m_creators.end()) {
                return it->second(config);
            } else {
                ROS_ERROR_STREAM("Specification type '" << spec_type_id << "' has not been registered, cannot create");
                return ObjSpecPtr{};
            }
        }
    private:
        ObjectSpecificationFactory() {}
        inline static std::map<std::string, std::function<ObjSpecPtr(const ObjectDimensionConfig&)>> m_creators;
    public:
        template <class SPEC_T>
        struct _Register {
            _Register(const std::string& spec_type_id) {
                ObjectSpecificationFactory::registerType<SPEC_T>(spec_type_id);
            }
        };
};

// Register an object specification type. `spec_type_id` is the string type (i.e. "box", "sphere", "cup", etc.). `SPEC_T` is the class.
// Statically constructs a _Register. To prevent naming confict, a subscript is prepended to the type.
#define REGISTER_OBJ_SPEC_TYPE(spec_type_id, SPEC_T) namespace {ObjectSpecificationFactory::_Register<SPEC_T> _##SPEC_T(spec_type_id);} 

// Object classes
struct Object {
    public:
        std::string id;
        std::shared_ptr<ObjectSpecification> spec;
        std::shared_ptr<PoseTracker> tracker;

        inline const static std::string s_default_class = "default";

        std::string class_id = s_default_class;
    private:
        geometry_msgs::Pose m_pose;
        Quaternions::Type m_orientation_type;

    public:
        Object() = delete;

        Object(const std::string& id_, const std::shared_ptr<ObjectSpecification>& spec_, Quaternions::Type orientation_type, const std::shared_ptr<PoseTracker>& tracker_ = nullptr, const std::string& class_id = s_default_class)  
            : id(id_)
            , spec(spec_)
            , tracker(tracker_)
            , class_id(class_id)
            , m_orientation_type(orientation_type)
        {
            setPoseToNeutral();
        }

        bool isStatic() const {return !tracker;}

        bool updatePose() {
            if (tracker) {
                return tracker->update(*this);
            }
            return false;
        }

        void setPose(const geometry_msgs::Pose& pose) {m_pose = pose;}

        const geometry_msgs::Pose& pose() const {return m_pose;}

        geometry_msgs::Pose graspPose() const {
            geometry_msgs::Pose grasp_pose = m_pose;
            grasp_pose.orientation = Quaternions::convert(Quaternions::convert(m_pose.orientation) * Quaternions::get(m_orientation_type));
            return grasp_pose;
        }

        void setPoseToNeutral() { m_pose = neutralPose(); }

        moveit_msgs::CollisionObject getCollisionObject(const std::string& frame_id) const {
            moveit_msgs::CollisionObject col_obj = spec->getCollisionObject(m_pose);
            col_obj.header.frame_id = frame_id;
            col_obj.id = id;
            return col_obj;
        }
};

using CollisionObjectVector = std::vector<moveit_msgs::CollisionObject>;

class ObjectGroup {
    public:
        ObjectGroup() = default;

        void createObjects(const ros::NodeHandle& nh, const std::string& ns, const std::shared_ptr<PoseTracker>& pose_tracker = nullptr);

        std::set<std::string> getIds() const {
            std::set<std::string> ids;
            for (auto v_type : m_objects) {ids.insert(v_type.first);}
            return ids;
        }

        inline std::size_t size() const {return m_objects.size();}

        inline Object& getObject(const std::string& id) { return m_objects.at(id); }
        inline const Object& getObject(const std::string& id) const { return m_objects.at(id);}

        inline std::vector<const Object*> getObjects() const {
            std::vector<const Object*> objects(m_objects.size());
            auto it = objects.begin();
            for (auto obj_it = m_objects.begin(); obj_it != m_objects.end(); ++obj_it)
                *(it++) = &obj_it->second;
            return objects;
        }
        inline std::vector<Object*> getObjects() {
            std::vector<Object*> objects(m_objects.size());
            auto it = objects.begin();
            for (auto obj_it = m_objects.begin(); obj_it != m_objects.end(); ++obj_it)
                *(it++) = &obj_it->second;
            return objects;
        }

        CollisionObjectVector getCollisionObjects(const std::string& frame_id) const {
            CollisionObjectVector collision_objs;
            for (const auto v_type : m_objects) collision_objs.push_back(v_type.second.getCollisionObject(frame_id));
            return collision_objs;
        }

        void insertObject(const Object& obj) {
            m_objects.insert(std::make_pair(obj.id, obj));
        }

        void insertObject(Object&& obj) {
            m_objects.insert(std::make_pair(obj.id, std::move(obj)));
        }

        void updatePoses() {
            for (auto& v_type : m_objects) v_type.second.updatePose();
        }

        bool updatePlanningScene(moveit::planning_interface::PlanningSceneInterface& pci, const std::string& planning_frame_id, bool ignore_static = true, bool update_poses = true);

        bool hasObject(const std::string& obj_id) const {return m_objects.find(obj_id) != m_objects.end();}
    protected:
        std::map<std::string, Object> m_objects;
};

}