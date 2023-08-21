#pragma once

#include "Object.h"

namespace TaskIt {

struct BoxObjectSpecification : public ObjectSpecification {
    double length = 0.05;
    double width = 0.05;
    double height = 0.05;

    BoxObjectSpecification() = delete;
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
            length = getDimensionConfigValue(config, "length");
            width = getDimensionConfigValue(config, "width");
            height = getDimensionConfigValue(config, "height");
        }
};
REGISTER_OBJ_SPEC_TYPE("box", BoxObjectSpecification);

struct SphereObjectSpecification : public ObjectSpecification {
    double radius = 0.05;

    SphereObjectSpecification() = delete;
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
            radius = getDimensionConfigValue(config, "radius");
        }
};
REGISTER_OBJ_SPEC_TYPE("sphere", SphereObjectSpecification);

struct CylinderObjectSpecification : public ObjectSpecification {
    double height = 0.05;
    double radius = 0.05;

    CylinderObjectSpecification() = delete;
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
            height = getDimensionConfigValue(config, "height");
            radius = getDimensionConfigValue(config, "radius");
        }
};
REGISTER_OBJ_SPEC_TYPE("cylinder", CylinderObjectSpecification);

}