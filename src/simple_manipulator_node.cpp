// System

// ROS
#include "ros/ros.h"

// ManipulationInterface
#include "ManipulatorNode.h"
#include "BaseActionPrimitives.h"
#include "Object.h"

using namespace ManipulationInterface;

class TestPoseTracker {
	public:
		static void update(const std::string& id, const geometry_msgs::Pose& pose) {

		}
};

static const std::string node_name = "simple_manipulator_node";

int main(int argc, char** argv) {
	
    ros::init(argc, argv, node_name);

	const std::string ee_link = "panda_link8";
	//std::shared_ptr<SimilarObjectGroup<TestPoseTracker>> obj_group;
	ManipulatorNode<UniqueObjectGroup<TestPoseTracker>, ActionPrimitives::SimpleGrasp> manipulator_node(node_name, "panda_arm", "panda_link0", nullptr, ActionPrimitives::SimpleGrasp(ee_link));

	manipulator_node.createWorkspace("workspace");

	ActionPrimitives::SimpleGrasp::msg_t::Request req;
	ActionPrimitives::SimpleGrasp::msg_t::Response res;
	manipulator_node.template callAction<ActionPrimitives::SimpleGrasp>(req, res);
	return 0;
}


