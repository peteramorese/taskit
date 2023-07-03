#pragma once

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <set>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>

#include "Quaternions.h"
#include "PoseTracker.h"

namespace TaskIt {


using ObjectConfig = std::map<std::string, float>;

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
        virtual void constructFromConfig(const ObjectConfig& config) = 0;
};

struct BoxObjectSpecification : SingleObjectSpecification {
    double length = 0.05;
    double width = 0.05;
    double height = 0.05;

    BoxObjectSpecification() = default;
    BoxObjectSpecification(const ObjectConfig& config) {constructFromConfig(config);}
    virtual float getLengthOffset() const override {return length / 2.0;}
    virtual float getWidthOffset() const override {return width / 2.0;}
    virtual float getHeightOffset() const override {return height / 2.0;}

    protected:
        virtual moveit_msgs::CollisionObject convert() const override {
            moveit_msgs::CollisionObject col_obj;
            col_obj.primitives.resize(1);
            auto& prim = col_obj.primitives[0];
            prim.type = prim.BOX;
            prim.dimensions = {length, width, height};
            return col_obj;
        }

        virtual void constructFromConfig(const ObjectConfig& config) override {
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
    virtual float getLengthOffset() const override {return radius;}
    virtual float getWidthOffset() const override {return radius;}
    virtual float getHeightOffset() const override {return radius;}

    protected:
        virtual moveit_msgs::CollisionObject convert() const override {
            moveit_msgs::CollisionObject col_obj;
            col_obj.primitives.resize(1);
            auto& prim = col_obj.primitives[0];
            prim.type = prim.SPHERE;
            prim.dimensions = {radius};
            return col_obj;
        }

        virtual void constructFromConfig(const ObjectConfig& config) override {
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
    virtual float getLengthOffset() const override {return radius;}
    virtual float getWidthOffset() const override {return radius;}
    virtual float getHeightOffset() const override {return height / 2.0;}

    protected:
        virtual moveit_msgs::CollisionObject convert() const override {
            moveit_msgs::CollisionObject col_obj;
            col_obj.primitives.resize(1);
            auto& prim = col_obj.primitives[0];
            prim.type = prim.CYLINDER;
            prim.dimensions = {height, radius};
            return col_obj;
        }

        virtual void constructFromConfig(const ObjectConfig& config) override {
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
struct Object {
    public:
        std::string id;
        std::shared_ptr<ObjectSpecification> spec;
        std::shared_ptr<PoseTracker> tracker;
        geometry_msgs::Pose pose;

    public:
        Object() = delete;

        Object(const std::string& id_, const std::shared_ptr<ObjectSpecification>& spec_, const ObjectConfig& config, const std::string& orientation_type, const std::shared_ptr<PoseTracker>& tracker_ = nullptr)  
            : id(id_)
            , spec(spec_)
            , tracker(tracker_)
        {
            setPoseFromConfig(config, orientation_type);
        }

        Object(const std::string& id_, const std::shared_ptr<ObjectSpecification>& spec_, const geometry_msgs::Pose& pose_, const std::shared_ptr<PoseTracker>& tracker_ = nullptr)  
            : id(id_)
            , spec(spec_)
            , tracker(tracker_)
            , pose(pose_)
            {}

        bool isStatic() const {return !tracker;}
        void updatePose() {if (tracker) tracker->update(*this);}

        void setPoseFromConfig(const ObjectConfig& config, const std::string& orientation_type) {
            if (config.find("x") != config.end()) {
                pose.position.x = config.at("x"); 
                pose.position.y = config.at("y");
                pose.position.z = config.at("z");
            } else {
                ROS_ASSERT_MSG(config.find("y") == config.end() && config.find("x") == config.end(), "Must provide (x, y, z) for the object, or leave all three fields empty");
                ROS_WARN_STREAM("No (x, y, z) coordinate provided for object '" << id <<"', assuming 0.0");
                pose.position.x = 0.0f;
                pose.position.y = 0.0f;
                pose.position.z = 0.0f;
            }
            pose.orientation = Quaternions::convert(Quaternions::get(orientation_type));
        }


        moveit_msgs::CollisionObject getCollisionObject(const std::string& frame_id) const {
            moveit_msgs::CollisionObject col_obj = spec->getCollisionObject(pose);
            col_obj.header.frame_id = frame_id;
            col_obj.id = id;
            return col_obj;
        }
};

using CollisionObjectVector = std::vector<moveit_msgs::CollisionObject>;

class ObjectGroup {
    public:
        ObjectGroup() = default;

        void createObjects(const ros::NodeHandle& nh, const std::string& ns, const std::string& frame_id, const std::shared_ptr<PoseTracker>& pose_tracker = nullptr);

        std::set<std::string> getIds() const {
            std::set<std::string> ids;
            for (auto v_type : m_objects) {ids.insert(v_type.first);}
            return ids;
        }

        inline std::size_t size() const {return m_objects.size();}

        //inline Object& getObject(const std::string& id) {return m_objects.at(id);}
        //inline const Object& getObject(const std::string& id) const {return m_objects.at(id);}
        inline Object& getObject(const std::string& id) {
            std::cout<<"b4 id: " << id <<std::endl;
            m_objects.at(id);
            std::cout<<"af" << std::endl;
            return m_objects.at(id);
            }
        inline const Object& getObject(const std::string& id) const {
            std::cout<<"b4 id: " << id <<std::endl;
            m_objects.at(id);
            std::cout<<"af" << std::endl;
            return m_objects.at(id);}

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

        void addObject(const std::string& id, const std::shared_ptr<ObjectSpecification>& spec) {
            this->m_objects.emplace(std::piecewise_construct, std::forward_as_tuple(id), std::forward_as_tuple(id, spec, geometry_msgs::Pose{}));
        }

        void addObject(const std::string& id, const std::shared_ptr<ObjectSpecification>& spec, const geometry_msgs::Pose& pose) {
            this->m_objects.emplace(std::piecewise_construct, std::forward_as_tuple(id), std::forward_as_tuple(id, spec, pose));
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

        bool hasObject(const std::string& obj_id) const {return m_objects.find(obj_id) != m_objects.end();}
    protected:
        std::map<std::string, Object> m_objects;
};

}