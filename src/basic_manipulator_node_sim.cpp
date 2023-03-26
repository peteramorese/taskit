// System

// ROS
#include "ros/ros.h"

// ManipulationInterface
#include "ManipulatorNode.h"
#include "BaseActionPrimitives.h"
#include "AdvancedActionPrimitives.h"
#include "Object.h"
#include "PoseTracker.h"
#include "Gripper.h"

using namespace ManipulationInterface;

class TestPoseTracker {
	public:
		static void update(const std::string& id, const geometry_msgs::Pose& pose) {

		}
};

static const std::string node_name = "basic_manipulator_node_sim";

int main(int argc, char** argv) {
	
    ros::init(argc, argv, node_name);

	const std::string ee_link = "panda_link8";
	
	// Make the simulation gripper handler
	std::shared_ptr<GripperHandler<GripperUse::Simulation>> gripper_handler = std::make_shared<GripperHandler<GripperUse::Simulation>>(MI_FRANKA_GRIPPER_TOPIC, 5.0);

	// Construct action primitives
	ActionPrimitives::Stow stow("stow");
	ActionPrimitives::SimpleGrasp<GripperUse::Simulation> grasp("grasp", gripper_handler, ee_link);
	ActionPrimitives::SimpleRelease<GripperUse::Simulation> release("release", gripper_handler);
	ActionPrimitives::TransitUp transit("transit", 5.0f, 1);
	ActionPrimitives::Transport transport("transport", 5.0f, 1, 0.05);

	ManipulatorNode<
	 	ActionPrimitives::Stow,
		ActionPrimitives::SimpleGrasp<GripperUse::Simulation>, 
		ActionPrimitives::SimpleRelease<GripperUse::Simulation>, 
		ActionPrimitives::TransitUp,
		ActionPrimitives::Transport
	> manipulator_node(node_name, "panda_arm", "panda_link0", 
		std::move(stow), 
		std::move(grasp), 
		std::move(release), 
		std::move(transit), 
		std::move(transport)
	);

	ros::WallDuration(1.0).sleep();

	// Pose tracker
	std::shared_ptr<SimulationPoseTracker> pose_tracker = std::make_shared<SimulationPoseTracker>();

	manipulator_node.createScene(pose_tracker);
	manipulator_node.updateEnvironment();

	manipulator_node.spawnAllActionServices();

	ros::waitForShutdown();
	return 0;
}


