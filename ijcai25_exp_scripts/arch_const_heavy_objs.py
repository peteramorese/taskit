#!/usr/bin/env python

'''
Rollout for the supplementary video. The task is simple - place box B1 at location L1. The box is in robot's hand.
'''
import sys
import time
from taskit.srv import Grasp, Release, Stow, Transit, UpdateEnv, SetObjectLocations
import rospy



def send_commands_to_robot(obj_id: str, start_loc: str, end_loc: str):
	""" 
	A helper function that run a set of pick and place for object
	"""

	s1 = send_transit_command_to_robot(start_loc)
	if s1:
		s2 = send_grasp_command_to_robot(obj_id)
	if s2:
		s3 = send_transport_command_to_robot(end_loc)
	if s3:
		s4 = send_release_command_to_robot(obj_id)

	if not (s1 & s2 & s3 & s4):
		print("Failed execution of the plan.")
		sys.exit(-1)

	# update the env after dropping
	update_env_status()


def stow_robot() -> bool:
	# helper function that updated the status (X, Y, Z) of the all the objects from vicon
	rospy.wait_for_service("/manipulator_node/action_primitive/stow")

	rospy.ServiceProxy("/manipulator_node/action_primitive/stow", Stow)()
	


def update_env_status() -> bool:
	# helper function that motion plans and stows the robot
	rospy.wait_for_service("/manipulator_node/action_primitive/update_environment")

	update_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/update_environment", UpdateEnv)
	t = update_handle(False)


def send_transport_command_to_robot(loc: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/transport")

	# create a handle for calling the service
	transport_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/transport", Transit)
	t = transport_handle(loc)
	return t.plan_success


def send_transit_command_to_robot(loc: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/linear_transit_up")

	# create a handle for calling the service
	transit_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/linear_transit", Transit) 
	t = transit_handle(loc)
	return t.plan_success


def send_transit_side_command_to_robot(loc: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/linear_transit_side")

	# create a handle for calling the service
	transit_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/linear_transit_side", Transit) 
	t = transit_handle(loc)
	return t.plan_success


def send_grasp_command_to_robot(obj_id: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/grasp")

	# create a handle for calling the service
	grasp_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/grasp", Grasp)
	# t = grasp_handle(obj_id)
	t = grasp_handle('')
	return t.mv_props.execution_success


def send_release_command_to_robot(obj_id: str) -> bool:
	#  convenience method that blocks until the service named is available
	rospy.wait_for_service("/manipulator_node/action_primitive/grasp")

	# create a handle for calling the service
	release_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/release", Release)
	# t = release_handle(obj_id)
	t = release_handle('')
	return t.mv_props.execution_success


def send_human_move_command(box: str , loc: str, act_name: str = None) -> bool:
	"""
	 Human Move command - send set object locations command.
	"""
	# create a handle for calling the service
	human_handle = rospy.ServiceProxy("/manipulator_node/action_primitive/set_object_locations", SetObjectLocations)
	t = human_handle([box], [loc])

	return t.success


if __name__ == "__main__":

	# init and update the status of all the boxes
	update_env_status()

	# pick and place B0 from l0 to l4
	send_transit_command_to_robot(loc='L0')
	send_grasp_command_to_robot(obj_id='B_0')
	send_transport_command_to_robot(loc='L4')
	send_release_command_to_robot(obj_id='B_0')

	# pick and place B1 from l4 to l0
	send_transit_command_to_robot(loc='L1')

	send_human_move_command('B_0', 'L0')

	send_grasp_command_to_robot(obj_id='B_1')
	send_transport_command_to_robot(loc='L5')
	send_release_command_to_robot(obj_id='B_1')

	send_transit_command_to_robot(loc='L0')
	send_grasp_command_to_robot(obj_id='B_0')
	send_transport_command_to_robot(loc='L4')
	send_release_command_to_robot(obj_id='B_0')

	# send_human_move_command('B_0', 'L4')
	# send_human_move_command('B_1', 'L5')

	# pick and place B2 from l2 to l6
	send_transit_side_command_to_robot(loc='L2')
	send_grasp_command_to_robot(obj_id='B_2')
	send_transport_command_to_robot(loc='L6')
	send_release_command_to_robot(obj_id='B_2')

	print("Done with executing.")