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

static const std::string node_name = "simple_manipulator_node";

int main(int argc, char** argv) {
	
    ros::init(argc, argv, node_name);

	const std::string ee_link = "panda_link8";
	
	// Make the simulation gripper handler
	std::shared_ptr<GripperHandler<GripperUse::Simulation>> gripper_handler = std::make_shared<GripperHandler<GripperUse::Simulation>>(MI_FRANKA_GRIPPER_TOPIC, 5.0);

	// Construct action primitives
	ActionPrimitives::Stow stow("stow");
	ActionPrimitives::SimpleGrasp<GripperUse::Simulation> grasp("grasp", gripper_handler, ee_link);
	ActionPrimitives::SimpleRelease<GripperUse::Simulation> release("release", gripper_handler);
	ActionPrimitives::TransitSide transit_side("transit_side", 5.0f, 1);
	ActionPrimitives::TransitUp transit_up("transit_up", 5.0f, 1);
	ActionPrimitives::LinearTransit linear_transit("linear_transit", 5.0f, 1, 0.05);
	ActionPrimitives::LinearTransitSide linear_transit_side("linear_transit_side", 5.0f, 1, 0.05);
	ActionPrimitives::Transport transport("transport", 5.0f, 1, 0.05);
	ActionPrimitives::LinearTransport linear_transport("linear_transport", 5.0f, 1, 0.05);

	ManipulatorNode<
	 	ActionPrimitives::Stow,
		ActionPrimitives::SimpleGrasp<GripperUse::Simulation>, 
		ActionPrimitives::SimpleRelease<GripperUse::Simulation>, 
		ActionPrimitives::TransitSide,
		ActionPrimitives::TransitUp,
		ActionPrimitives::LinearTransit,
		ActionPrimitives::LinearTransitSide,
		ActionPrimitives::Transport,
		ActionPrimitives::LinearTransport
	> manipulator_node(node_name, "panda_arm", "panda_link0", 
		std::move(stow), 
		std::move(grasp), 
		std::move(release), 
		std::move(transit_side), 
		std::move(transit_up), 
		std::move(linear_transit), 
		std::move(linear_transit_side),
		std::move(transport),
		std::move(linear_transport)
	);

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


