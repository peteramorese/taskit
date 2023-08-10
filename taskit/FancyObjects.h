#pragma once

#include "Object.h"

namespace TaskIt {

struct CupObjectSpecification : public ObjectSpecification {
    double radius;
    double height;
    double handle_length;
    double handle_width;
    double flange_height;
    double flange_width;

    CupObjectSpecification() = delete;
    CupObjectSpecification(const ObjectDimensionConfig& config) {constructFromConfig(config);}
    virtual float getLengthOffset() const override {return radius;}
    virtual float getWidthOffset() const override {return radius;}
    virtual float getHeightOffset() const override {return height / 2.0;}

    protected:
        virtual moveit_msgs::CollisionObject getCollisionObject(const geometry_msgs::Pose& pose) const override {
            moveit_msgs::CollisionObject col_obj;
            // Set the gobal pose
            col_obj.pose = pose;

            col_obj.primitives.resize(3);
            col_obj.primitive_poses.resize(3);

            // Well primitive
            auto& well = col_obj.primitives[0];
            well.type = well.CYLINDER;
            well.dimensions = {height, radius};
            // Well pose
            col_obj.primitive_poses[0] = neutralPose();

            // Flange primitive
            auto& flange = col_obj.primitives[1];
            flange.type = flange.CYLINDER;
            flange.dimensions = {flange_height, radius + flange_width};
            // Flange pose
            col_obj.primitive_poses[1] = neutralPose();
            col_obj.primitive_poses[1].position.z = (flange_height - height) / 2.0;

            // Handle primitive
            auto& handle = col_obj.primitives[2];
            handle.type = handle.BOX;
            handle.dimensions = {radius + handle_length, handle_width, height};
            // Flange pose
            col_obj.primitive_poses[2] = neutralPose();
            col_obj.primitive_poses[2].position.x = (radius + handle_length) / 2.0; // Handle points out +x
            

            // Primitive pose
            return col_obj;
        }

        virtual void constructFromConfig(const ObjectDimensionConfig& config) override {
            height = getDimensionConfigValue(config, "height");
            radius = getDimensionConfigValue(config, "radius");
            handle_length = getDimensionConfigValue(config, "handle_length");
            handle_width = getDimensionConfigValue(config, "handle_width");
            flange_height = getDimensionConfigValue(config, "flange_height");
            flange_width = getDimensionConfigValue(config, "flange_width");
        }
};
REGISTER_OBJ_SPEC_TYPE("cup", CupObjectSpecification);

}