# TaskIt

ROS package for abstracting a robotic manipulation domain and manipulation action primitives for discrete task planning. Currently supports only the Franka Emika Panda manipulator, however the C++ library is abstracted to support any manipulator.

**NOTE:** Not affiliated with MoveIt, however this package heavily utilizes MoveIt for low-level motion planning and visualization. 

[Code Examples](./examples.html) | [Making Objects in Vicon Tracker](./making_objects.html)



## Built-In Features

1. Action/Motion primitive abstraction into `rosservice` call-and-response
2. Configuration file for abstract environments and object specifications
3. Abstraction for integration with object pose trackers such as Vicon Motion Tracker

#### Action primitives
 - Stow (move the arm to the default configuration)
 - SimpleGrasp (Grab an object)
 - SimpleRelease (Release an object)
 - Transit [All/Up/Side] (Moves the end effector to either an empty location or to the pre-condition grasping pose if an object is in the location)
 - Transport (Moves the end effector to an empty location if it is grabbing an object)
 - Linear Transit [All/Up/Side] (Transit with a safe linear approach and retreat)
 - Linear Transport (Transport with a safe linear approach and retreat)

#### Config file abstraction for
 - Discrete locations
 - Workspace collision objects
 - Manipulatable objects
 - Other relevant properties for the working manipulator

 

---

 #### Contact
Peter Amorese | peter.amorese@colorado.edu