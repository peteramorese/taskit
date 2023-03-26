# Manipulation Interface

ROS package for abstracting a robotic manipulation domain and manipulation action primitives.

## Built-In Features

Action primitives:
 - Stow (move the arm to the default configuration)
 - SimpleGrasp (Grab an object)
 - SimpleRelease (Release an object)
 - Transit [All/Up/Side] (Moves the end effector to either an empty location or to the pre-condition grasping pose if an object is in the location)
 - Transport (Moves the end effector to an empty location if it is grabbing an object)
 - Linear Transit [All/Up/Side] (Transit with a safe linear approach and retreat)
 - Linear Transport (Transport with a safe linear approach and retreat)

ROS Param Config file abstraction for:
 - Discrete locations
 - Workspace collision objects
 - Dynamic manipulatable objects
 - Other relevant properties for the working manipulator