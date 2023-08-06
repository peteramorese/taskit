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

## Creating a container
To create your container, run:
`docker compose up`

The docker compose file automatically builds and caches the image. The root directory of `taskit` is mounted inside `ws/src/taskit` in the container so changes made to the local repository are reflected inside the container and visa versa.

To spin up a container and rebuild the image:
`docker compose up --build`

## Running the manipulator node
Once inside the container, you can launch the manipulator node with
`roslaunch taskit manipulator_node.launch`
Use the following flags for your specific experiment:
 - `sim:=<true, false>`: `true` (default) for running an experiment in simulation only, `false` for running an experiment with a real robot connected
 - `pose_tracker:=<simulation, vrpn>`: `simulation` (default) for no external pose tracking/update information, `vrpn` for observing pose information from Vicon Tracker
 - `config_directory:=<my_config_directory>`: Specify a directory where the `environment.yaml`, `workspace.yaml`, and `objects.yaml` files appear. Defaults to `config`