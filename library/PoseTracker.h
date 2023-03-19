#pragma once

namespace ManipulationInterface {

// Forward declarations
class Object;

class PoseTracker {
    public:
        virtual void update(Object& object) const = 0;

    protected:
        PoseTracker() {}
};

class SimulationPoseTracker : public PoseTracker {
    public:
        virtual void update(Object& object) const override {}

};

}