// System

// ROS
#include "ros/ros.h"

// ManipulationInterface
#include "ManipulatorNode.h"
#include "BaseActionPrimitives.h"
#include "Object.h"
#include "PoseTracker.h"
#include "Gripper.h"

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
	
	// Make the simulation gripper handler
	std::shared_ptr<GripperHandler<GripperUse::Simulation>> gripper_handler = std::make_shared<GripperHandler<GripperUse::Simulation>>(MI_FRANKA_GRIPPER_TOPIC, 5.0);

	// Construct action primitives
	ActionPrimitives::SimpleGrasp<GripperUse::Simulation> grasp("grasp", gripper_handler, ee_link);
	ActionPrimitives::SimpleRelease<GripperUse::Simulation> release("release", gripper_handler);
	ActionPrimitives::Transit transit("transit", 5.0f, 1);

	ManipulatorNode<
		ActionPrimitives::SimpleGrasp<GripperUse::Simulation>, 
		ActionPrimitives::SimpleRelease<GripperUse::Simulation>, 
		ActionPrimitives::Transit
	> manipulator_node(node_name, "panda_arm", "panda_link0", std::move(grasp), std::move(release), std::move(transit));

	ros::WallDuration(1.0).sleep();

	// Pose tracker
	std::shared_ptr<SimulationPoseTracker> pose_tracker = std::make_shared<SimulationPoseTracker>();

	manipulator_node.createScene(pose_tracker);
	manipulator_node.updateEnvironment();

	manipulator_node.spawnAllActionServices();
	//manipulator_node.template callActionByType<ActionPrimitives::SimpleGrasp>(req, res);
	//manipulator_node.template callActionByIndex<0>(req, res);

	ros::waitForShutdown();
	return 0;
}


