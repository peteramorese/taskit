# TaskIt

ROS package for abstracting a robotic manipulation domain and manipulation action primitives for discrete task planning. Currently supports only the Franka Emika Panda manipulator, however the C++ library is abstracted to support any manipulator.

**NOTE:** Not affiliated with MoveIt, however this package heavily utilizes MoveIt for low-level motion planning and visualization. 

## Built-In Features

Action primitives:
 - Stow (move the arm to the default configuration)
 - SimpleGrasp (Grab an object)
 - SimpleRelease (Release an object)
 - Transit [All/Up/Side] (Moves the end effector to either an empty location or to the pre-condition grasping pose if an object is in the location)
 - Transport (Moves the end effector to an empty location if it is grabbing an object)
 - Linear Transit [All/Up/Side] (Transit with a safe linear approach and retreat)
 - Linear Transport (Transport with a safe linear approach and retreat)

Config file abstraction for:
 - Discrete locations
 - Workspace collision objects
 - Manipulatable objects
 - Other relevant properties for the working manipulator


# Instructions to run the code

1. roslaunch manipulator_node.launch sim:=false pose\_tracker:={vrpn, sim}

vrpn = Vicon
sim = Hardcoded values

config: Change env.yaml and config.yml
