// System

// ROS
#include "ros/ros.h"

// ManipulationInterface
#include "library/ManipulatorNode.h"
#include "library/BaseActionPrimitives.h"
#include "library/Object.h"


class TestPoseTracker {
	void update(const std::string& id, const geometry_msgs::Pose& pose) {

	}
};

int main(int argc, char** argv) {
	ManipulationInterface<SimilarObjectGroup<TestPoseTracker>, SimpleGrasp, SimpleRelease, Transit>;
	return 0;
}


