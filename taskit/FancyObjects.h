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

struct BinObjectSpecification : public ObjectSpecification {
    double length;
    double width;
    double height;
    double base_thickness;
    double front_wall_thickness;
    double side_wall_thickness;

    BinObjectSpecification() = delete;
    BinObjectSpecification(const ObjectDimensionConfig& config) {constructFromConfig(config);}
    virtual float getLengthOffset() const override {return length / 2.0;}
    virtual float getWidthOffset() const override {return width / 2.0;}
    virtual float getHeightOffset() const override {return height / 2.0;}

    protected:
        virtual moveit_msgs::CollisionObject getCollisionObject(const geometry_msgs::Pose& pose) const override {
            moveit_msgs::CollisionObject col_obj;
            // Set the gobal pose
            col_obj.pose = pose;

            col_obj.primitives.resize(5);
            col_obj.primitive_poses.resize(5);

            // Base primitive
            auto& base = col_obj.primitives[0];
            base.type = base.BOX;
            base.dimensions = {length, width, base_thickness};
            // Base pose
            col_obj.primitive_poses[0] = neutralPose();
            col_obj.primitive_poses[0].position.z = (base_thickness - height) / 2.0;

            // Front & back wall primitive
            auto& front_wall = col_obj.primitives[1];
            front_wall.type = front_wall.BOX;
            front_wall.dimensions = {front_wall_thickness, width, height};
            auto& back_wall = col_obj.primitives[2];
            back_wall.type = front_wall.BOX;
            back_wall.dimensions = {front_wall_thickness, width, height};
            // Front & back wall pose
            col_obj.primitive_poses[1] = neutralPose();
            col_obj.primitive_poses[1].position.x = -(front_wall_thickness - length) / 2.0;
            col_obj.primitive_poses[2] = neutralPose();
            col_obj.primitive_poses[2].position.x =  (front_wall_thickness - length) / 2.0;

            // Left & right wall primitive
            auto& left_wall = col_obj.primitives[3];
            left_wall.type = left_wall.BOX;
            left_wall.dimensions = {length, side_wall_thickness, height};
            auto& right_wall = col_obj.primitives[4];
            right_wall.type = left_wall.BOX;
            right_wall.dimensions = {length, side_wall_thickness, height};
            // Left & right wall pose
            col_obj.primitive_poses[3] = neutralPose();
            col_obj.primitive_poses[3].position.y =  (side_wall_thickness - width) / 2.0; 
            col_obj.primitive_poses[4] = neutralPose();
            col_obj.primitive_poses[4].position.y = -(side_wall_thickness - width) / 2.0; 
            
            return col_obj;
        }

        virtual void constructFromConfig(const ObjectDimensionConfig& config) override {
            length = getDimensionConfigValue(config, "length");
            width = getDimensionConfigValue(config, "width");
            height = getDimensionConfigValue(config, "height");
            base_thickness = getDimensionConfigValue(config, "base_thickness");
            front_wall_thickness = getDimensionConfigValue(config, "front_wall_thickness");
            side_wall_thickness = getDimensionConfigValue(config, "side_wall_thickness");
        }
};
REGISTER_OBJ_SPEC_TYPE("bin", BinObjectSpecification);

struct LidObjectSpecification : public ObjectSpecification {
    double lid_length;
    double lid_width;
    double lid_height;
    double handle_length;
    double handle_width;
    double handle_height;

    LidObjectSpecification() = delete;
    LidObjectSpecification(const ObjectDimensionConfig& config) {constructFromConfig(config);}
    virtual float getLengthOffset() const override {return handle_length / 2.0;}
    virtual float getWidthOffset() const override {return handle_width / 2.0;}
    virtual float getHeightOffset() const override {return handle_height / 2.0;}

    protected:
        virtual moveit_msgs::CollisionObject getCollisionObject(const geometry_msgs::Pose& pose) const override {
            moveit_msgs::CollisionObject col_obj;
            // Set the gobal pose
            col_obj.pose = pose;

            col_obj.primitives.resize(2);
            col_obj.primitive_poses.resize(2);

            // Lid primitive
            auto& lid = col_obj.primitives[0];
            lid.type = lid.BOX;
            lid.dimensions = {lid_length, lid_width, lid_height};
            // Lid pose
            col_obj.primitive_poses[0] = neutralPose();
            col_obj.primitive_poses[0].position.z = -(handle_height + lid_height) / 2.0;

            // Handle primitive
            auto& handle = col_obj.primitives[1];
            handle.type = handle.BOX;
            handle.dimensions = {handle_length, handle_width, handle_height};
            // Handle pose
            col_obj.primitive_poses[1] = neutralPose();

            return col_obj;
        }

        virtual void constructFromConfig(const ObjectDimensionConfig& config) override {
            lid_length = getDimensionConfigValue(config, "lid_length");
            lid_width = getDimensionConfigValue(config, "lid_width");
            lid_height = getDimensionConfigValue(config, "lid_height");
            handle_length = getDimensionConfigValue(config, "handle_length");
            handle_width = getDimensionConfigValue(config, "handle_width");
            handle_height = getDimensionConfigValue(config, "handle_height");
        }
};
REGISTER_OBJ_SPEC_TYPE("lid", LidObjectSpecification);

}