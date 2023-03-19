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
	ActionPrimitives::SimpleGrasp grasp("grasp", ee_link);
	ActionPrimitives::Transit transit("transit", 5.0f, 1);
	ManipulatorNode<ActionPrimitives::SimpleGrasp, ActionPrimitives::Transit> manipulator_node(node_name, "panda_arm", "panda_link0", std::move(grasp), std::move(transit));

	ros::WallDuration(1.0).sleep();

	manipulator_node.createScene();
	manipulator_node.updateEnvironment();

	ActionPrimitives::SimpleGrasp::msg_t::Request req;
	ActionPrimitives::SimpleGrasp::msg_t::Response res;
	manipulator_node.spawnAllActionServices();
	//manipulator_node.template callActionByType<ActionPrimitives::SimpleGrasp>(req, res);
	//manipulator_node.template callActionByIndex<0>(req, res);

	ros::waitForShutdown();
	return 0;
}


