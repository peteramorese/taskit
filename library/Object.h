#pragma once

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

#include "Quaternions.h"

namespace ManipulationInterface {


using ObjectConfig = std::map<std::string, float>;

// Object specifications

enum class ObjectPrimitive {
    Box,
    Sphere,
    Cylinder
};

static ObjectPrimitive getObjectPrimitiveType(const std::string& str) {
    if (str == "box") return ObjectPrimitive::Box;
    if (str == "sphere") return ObjectPrimitive::Sphere;
    if (str == "cylinder") return ObjectPrimitive::Cylinder;
    ROS_ERROR("Unrecognized primitive type: " + str);
}

struct ObjectSpecification {
    public: 
        // Grip properties
        double grip_force = 50.0;
        double grip_speed = 0.1;
        double grip_width_closed = 0.05;
        double grip_width_open = 0.1;

        double end_effector_offset = 0.08;
        double placement_safety_distance = 0.01;

    public:
        virtual moveit_msgs::CollisionObject getCollisionObject(const geometry_msgs::Pose& pose) const = 0;

};

struct SingleObjectSpecification : ObjectSpecification {
    public:
        virtual moveit_msgs::CollisionObject getCollisionObject(const geometry_msgs::Pose& pose) const override {
            moveit_msgs::CollisionObject col_obj = convert();
            col_obj.primitive_poses.resize(1);
            col_obj.primitive_poses[0] = pose;
            return col_obj;
        }

    protected:
        virtual moveit_msgs::CollisionObject convert() const = 0;
        virtual constructFromConfig(const ObjectConfig& config) = 0;
};

struct BoxObjectSpecification : SingleObjectSpecification {
    double length = 0.05;
    double width = 0.05;
    double height = 0.05;

    BoxObjectSpecification() = default;
    BoxObjectSpecification(const ObjectConfig& config) {constructFromConfig(config);}

    protected:
        virtual moveit_msgs::CollisionObject convert() const override {
            moveit_msgs::CollisionObject col_obj;
            col_obj.primitives.resize(1);
            auto& prim = col_obj.primitives[0];
            prim.type = prim.BOX;
            prim.dimensions = {length, width, height};
            return col_obj;
        }

        virtual constructFromConfig(const ObjectConfig& config) override {
            try {
                length = config.at("l");
                width = config.at("w");
                height = config.at("h");
            } catch (const std::exception& e) {
                ROS_ERROR("Config parameters do not match");
            }
        }
};

struct SphereObjectSpecification : SingleObjectSpecification {
    double radius = 0.05;

    SphereObjectSpecification() = default;
    SphereObjectSpecification(const ObjectConfig& config) {constructFromConfig(config);}

    protected:
        virtual moveit_msgs::CollisionObject convert() const override {
            moveit_msgs::CollisionObject col_obj;
            col_obj.primitives.resize(1);
            auto& prim = col_obj.primitives[0];
            prim.type = prim.SPHERE;
            prim.dimensions = {radius};
            return col_obj;
        }

        virtual constructFromConfig(const ObjectConfig& config) override {
            try {
                radius = config.at("r");
            } catch (const std::exception& e) {
                ROS_ERROR("Config parameters do not match");
            }
        }
};

struct CylinderObjectSpecification : SingleObjectSpecification {
    double height = 0.05;
    double radius = 0.05;

    CylinderObjectSpecification() = default;
    CylinderObjectSpecification(const ObjectConfig& config) {constructFromConfig(config);}

    protected:
        virtual moveit_msgs::CollisionObject convert() const override {
            moveit_msgs::CollisionObject col_obj;
            col_obj.primitives.resize(1);
            auto& prim = col_obj.primitives[0];
            prim.type = prim.CYLINDER;
            prim.dimensions = {height, radius};
            return col_obj;
        }

        virtual constructFromConfig(const ObjectConfig& config) override {
            try {
                height = config.at("h");
                radius = config.at("r");
            } catch (const std::exception& e) {
                ROS_ERROR("Config parameters do not match");
            }
        }
};

static std::shared_ptr<ObjectSpecification> makeObjectSpecification(ObjectPrimitive primitive, const ObjectConfig& config) {
    switch (primitive) {
        case ObjectPrimitive::Box: return std::make_shared<BoxObjectSpecification>(config);
        case ObjectPrimitive::Sphere: return std::make_shared<SphereObjectSpecification>(config);
        case ObjectPrimitive::Cylinder: return std::make_shared<CylinderObjectSpecification>(config);
    }
    ROS_ERROR("Unrecognized primitive type");
    return std::shared_ptr<ObjectSpecification>{};
}

static std::shared_ptr<ObjectSpecification> makeObjectSpecification(const std::string& primitive_str, const ObjectConfig& config) {
    return makeObjectSpecification(getObjectPrimitiveType(primitive_str), config);
}

// Object classes
template <class POSE_TRACKER_T>
struct Object {
    public:
        std::string id;
        std::shared_ptr<ObjectSpecification> spec;
        geometry_msgs::Pose pose;

    public:
        Object() = delete;

        Object(const std::string& id_, const std::shared_ptr<ObjectSpecification>& spec_, const ObjectConfig& config, const std::string& orientation_type)  
            : id(id_)
            , spec(spec_)
        {
            setPoseFromConfig(config, orientation_type);
        }

        Object(const std::string& id_, const std::shared_ptr<ObjectSpecification>& spec_, const geometry_msgs::Pose& pose_)  
            : id(id_)
            , spec(spec_)
            , pose(pose_)
            {}

        void updatePose() {POSE_TRACKER_T::update(id, pose);}
        void setPoseFromConfig(const ObjectConfig& config, const std::string& orientation_type) {
            pose.position.x = config.at("x");
            pose.position.y = config.at("y");
            pose.position.z = config.at("z");
            pose.orientation = Quaternions::convert(Quaternions::get(orientation_type));
        }


        moveit_msgs::CollisionObject getCollisionObject() const {
            moveit_msgs::CollisionObject col_obj = spec->getCollisionObject(pose);
            col_obj.id = id;
            return col_obj;
        }
};

template <class POSE_TRACKER_T>
class ObjectGroup {
    public:
        typedef POSE_TRACKER_T pose_tracker_t;
        using ObjectType = Object<POSE_TRACKER_T>;
    protected:
        ObjectGroup() = default;

    public:
        std::set<std::string> getIds() const {
            std::set<std::string> ids;
            for (auto v_type : m_objects) {ids.insert(v_type.first);}
            return ids;
        }

        inline const ObjectType& getObject(const std::string& id) const {return m_objects.at(id);}
        inline const std::map<std::string, ObjectType>& getObjects() const {return m_objects;}

        void insertObject(const ObjectType& obj) {
            m_objects.insert(std::make_pair(obj.id, obj));
        }

        void insertObject(ObjectType&& obj) {
            m_objects.insert(std::make_pair(obj.id, std::move(obj)));
        }

        void updatePoses() {
            for (auto& v_type : m_objects) v_type.second.updatePose();
        }


    protected:
        std::map<std::string, ObjectType> m_objects;
};

template <class POSE_TRACKER_T>
class SimilarObjectGroup : public ObjectGroup<POSE_TRACKER_T> {
    public:
        SimilarObjectGroup(const std::shared_ptr<ObjectSpecification>& spec) 
            : m_spec(spec)
        {
        }

        void addObject(const std::string& id) {
            m_objects.emplace(std::piecewise_construct, std::forward_as_tuple(id), std::forward_as_tuple(id, m_spec, geometry_msgs::Pose{}))
        }

        void addObject(const std::string& id, const geometry_msgs::Pose& pose) {
            m_objects.emplace(std::piecewise_construct, std::forward_as_tuple(id), std::forward_as_tuple(id, m_spec, pose))
        }

        virtual const ObjectSpecification& getSpec() const {return *m_spec;}

    private:
        std::shared_ptr<ObjectSpecification> m_spec;
};

class UniqueObjectGroup : public ObjectGroup<POSE_TRACKER_T> {
    public:
        UniqueObjectGroup() 
            {}

        void addObject(const std::string& id, const std::shared_ptr<ObjectSpecification>& spec) {
            m_objects.emplace(std::piecewise_construct, std::forward_as_tuple(id), std::forward_as_tuple(id, spec, geometry_msgs::Pose{}))
        }

        void addObject(const std::string& id, const std::shared_ptr<ObjectSpecification>& spec, const geometry_msgs::Pose& pose) {
            m_objects.emplace(std::piecewise_construct, std::forward_as_tuple(id), std::forward_as_tuple(id, spec, pose))
        }

        void operator+=(const ObjectType& obj) {
            m_objects.insert(std::make_pair(obj.id, obj));
        }

        void operator+=(ObjectType&& obj) {
            m_objects.insert(std::make_pair(obj.id, std::move(obj)));
        }

        void operator+=(const ObjectGroup<POSE_TRACKER_T>& obj_group) {
            for (const auto& v_type : obj_group.getObjects()) {
                m_objects.insert(std::make_pair(v_type.first, v_type.second));
            }
        }
};

}