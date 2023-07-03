#include "ros/ros.h"
#include "TaskIt.h"

using namespace TaskIt;

static const std::string node_name = "advanced_manipulator_node_real";

int main(int argc, char** argv) {
	
    ros::init(argc, argv, node_name);
    std::shared_ptr<ros::NodeHandle> node_handle = std::make_shared<ros::NodeHandle>("~");

	const std::string ee_link = "panda_link8";
	
	// Make the simulation gripper handler
	std::shared_ptr<GripperHandler<GripperUse::FrankaHand>> gripper_handler = std::make_shared<GripperHandler<GripperUse::FrankaHand>>(MI_FRANKA_GRIPPER_TOPIC, 5.0);

	// Construct action primitives
	ActionPrimitives::Stow stow("stow");
	ActionPrimitives::SimpleGrasp<GripperUse::FrankaHand> grasp("grasp", gripper_handler, ee_link);
	ActionPrimitives::SimpleRelease<GripperUse::FrankaHand> release("release", gripper_handler);
	ActionPrimitives::TransitSide transit_side("transit_side", 5.0f, 1);
	ActionPrimitives::TransitUp transit_up("transit_up", 5.0f, 1);
	ActionPrimitives::LinearTransit linear_transit("linear_transit", 5.0f, 1, 0.05);
	ActionPrimitives::LinearTransitSide linear_transit_side("linear_transit_side", 5.0f, 1, 0.05);
	ActionPrimitives::Transport transport("transport", 5.0f, 1, 0.05);
	ActionPrimitives::LinearTransport linear_transport("linear_transport", 5.0f, 1, 0.05);

	ManipulatorNode<
	 	decltype(stow),
		decltype(grasp), 
		decltype(release), 
		decltype(transit_side),
		decltype(transit_up),
		decltype(linear_transit),
		decltype(linear_transit_side),
		decltype(transport),
		decltype(linear_transport)
	> manipulator_node(node_handle, node_name, "panda_arm", "panda_link0", 
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

	ros::waitForShutdown();
	return 0;
}


