// System

// ROS
#include "ros/ros.h"

// ManipulationInterface
#include "ManipulatorNode.h"
#include "BaseActionPrimitives.h"
#include "Object.h"

using namespace ManipulationInterface;

class TestPoseTracker {
	void update(const std::string& id, const geometry_msgs::Pose& pose) {

	}
};

int main(int argc, char** argv) {

	const std::string ee_link = "panda_link8";
	std::shared_ptr<SimilarObjectGroup<TestPoseTracker>> obj_group;
	ManipulatorNode<SimilarObjectGroup<TestPoseTracker>, ActionPrimitives::SimpleGrasp> manipulator_node(argc, argv, "panda_arm", "panda_link8", obj_group, ActionPrimitives::SimpleGrasp(ee_link));
	manipulator_node.template callAction<ActionPrimitives::SimpleGrasp>();
	return 0;
}


