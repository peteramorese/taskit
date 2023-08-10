#pragma once

#include "Object.h"

namespace TaskIt {

struct BoxObjectSpecification : public ObjectSpecification {
    double length = 0.05;
    double width = 0.05;
    double height = 0.05;

    BoxObjectSpecification() = default;
    BoxObjectSpecification(const ObjectDimensionConfig& config) {constructFromConfig(config);}
    virtual float getLengthOffset() const override {return length / 2.0;}
    virtual float getWidthOffset() const override {return width / 2.0;}
    virtual float getHeightOffset() const override {return height / 2.0;}

    protected:
        virtual moveit_msgs::CollisionObject getCollisionObject(const geometry_msgs::Pose& pose) const override {
            moveit_msgs::CollisionObject col_obj;
            // Set the gobal pose
            col_obj.pose = pose;

            // Primitive type
            col_obj.primitives.resize(1);
            auto& prim = col_obj.primitives[0];
            prim.type = prim.BOX;
            prim.dimensions = {length, width, height};

            // Primitive pose
            col_obj.primitive_poses.resize(1);
            col_obj.primitive_poses[0] = neutralPose();
            return col_obj;
        }

        virtual void constructFromConfig(const ObjectDimensionConfig& config) override {
            try {
                length = config.at("l");
                width = config.at("w");
                height = config.at("h");
            } catch (const std::exception& e) {
                ROS_ERROR("Config parameters do not match (need 'l', 'w', 'h' for box)");
            }
        }

        //REGISTER_OBJ_SPEC_TYPE("box", BoxObjectSpecification);
};
REGISTER_OBJ_SPEC_TYPE("box", BoxObjectSpecification);

struct SphereObjectSpecification : public ObjectSpecification {
    double radius = 0.05;

    SphereObjectSpecification() = default;
    SphereObjectSpecification(const ObjectDimensionConfig& config) {constructFromConfig(config);}
    virtual float getLengthOffset() const override {return radius;}
    virtual float getWidthOffset() const override {return radius;}
    virtual float getHeightOffset() const override {return radius;}

    protected:
        virtual moveit_msgs::CollisionObject getCollisionObject(const geometry_msgs::Pose& pose) const override {
            moveit_msgs::CollisionObject col_obj;
            // Set the gobal pose
            col_obj.pose = pose;

            // Primitive type
            col_obj.primitives.resize(1);
            auto& prim = col_obj.primitives[0];
            prim.type = prim.SPHERE;
            prim.dimensions = {radius};

            // Primitive pose
            col_obj.primitive_poses.resize(1);
            col_obj.primitive_poses[0] = neutralPose();
            return col_obj;
        }

        virtual void constructFromConfig(const ObjectDimensionConfig& config) override {
            try {
                radius = config.at("r");
            } catch (const std::exception& e) {
                ROS_ERROR("Config parameters do not match (need 'r' for sphere)");
            }
        }
};
REGISTER_OBJ_SPEC_TYPE("sphere", SphereObjectSpecification);

struct CylinderObjectSpecification : public ObjectSpecification {
    double height = 0.05;
    double radius = 0.05;

    CylinderObjectSpecification() = default;
    CylinderObjectSpecification(const ObjectDimensionConfig& config) {constructFromConfig(config);}
    virtual float getLengthOffset() const override {return radius;}
    virtual float getWidthOffset() const override {return radius;}
    virtual float getHeightOffset() const override {return height / 2.0;}

    protected:
        virtual moveit_msgs::CollisionObject getCollisionObject(const geometry_msgs::Pose& pose) const override {
            moveit_msgs::CollisionObject col_obj;
            // Set the gobal pose
            col_obj.pose = pose;

            // Primitive type
            col_obj.primitives.resize(1);
            auto& prim = col_obj.primitives[0];
            prim.type = prim.CYLINDER;
            prim.dimensions = {height, radius};

            // Primitive pose
            col_obj.primitive_poses.resize(1);
            col_obj.primitive_poses[0] = neutralPose();
            return col_obj;
        }

        virtual void constructFromConfig(const ObjectDimensionConfig& config) override {
            try {
                height = config.at("h");
                radius = config.at("r");
            } catch (const std::exception& e) {
                ROS_ERROR("Config parameters do not match (need 'h', 'r' for cylinder)");
            }
        }
};
REGISTER_OBJ_SPEC_TYPE("cylinder", CylinderObjectSpecification);

}