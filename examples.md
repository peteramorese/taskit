# Examples

Here are a few examples that explain the functionality of TaskIt, and how to interface with this project in various instances. The examples build off each other.

## Using TaskIt from the command-line using `rosservice` (Beginner)
Calling action primitives from the command line is as simple as 
1. setting up your workspace, abstracted environment, and object specifications in some config files, 
2. launching the `manipulator_node`,
3. using `rosservice` to request ROS services from the command line

#### Configuration files
Let's begin with creating our configuration files. The TaskIt launch structure looks under the `$(find taskit)/environment_config/` directory for all of the config files. Let's make a workspace file `workspace.yaml` that defines static collision obstacles around our space. 

```yaml
# Create an ID for each obstacle
object_ids:
  - "ground"

# Specify the object primitive (box/cylinder/sphere)
object_types:
  - "box"

# For each ID in `object_ids`, specify the following data
ground: {
  x: 0.0,   # center point
  y: 0.0,
  z: -0.05,
  l: 1.5,   # primitive dimensions
  w: 1.5,
  h: 0.1,
}

# Corresponding to each ID, apply a simple orientation if you would like, the default is `up_x`
object_orientation_types:
  - up_x # for groud
```

For `"box"` primitive types, the dimension fields are `l` (length), `w` (width), `h` (height). For cylinder, specify `r` (radius) and `h` (height), and for sphere, just specify `r` (radius). The `object_orientation_types` field can be used to apply: 
- a single 90 degree yaw using `up_y`,
- a single 90 degree roll using `side_x`,
- a 90 degree yaw and roll using `side_y`
to the default orientation of the object primitive. Currently, we do not support custom orientations in the abstraction layer.

Now lets make the environment file `environment.yaml` that defines the discrete-location abstraction of our space.

```yaml
# Create an ID for each location
location_names: 
  - "L1"
  - "L2"

# For each location ID, specify the following data
L1: { 
  x: 0.30,  # center point
  y: 0.20,
  z: 0.09, 
  r: 0.20,  # detection radius
}

L2: { 
  x: 0.30,
  y: -0.20,
  z: 0.09, 
  r: 0.20, 
}

# Corresponding to each ID, specify the placing-orientation type
location_orientation_types:
  - up_x # for L1
  - up_x # for L2
```

Each location ID must have a set of data and an orientation type (as shown above). 

The detection radius `r` field in the location data defines the region for which an object is considered "at that location". If an object's center is within `r` from a location's center, the system reads that object as occupying the location. **Note:** If multiple objects are within `r` distance, the closest one is used.

The `location_orientation_type` defines the relative orientation that the object should be placed in. For example, if you want the robot to place the object sideways in `L2`, then the orientation type would be `side_x`. This feature is particularly useful for stacking and arch problems.

Finally, lets make the objects file `objects.yaml` that defines the obstacles that can be manipulated and moved around our space.

```yaml
# Create an ID for each object
object_ids:
  - blue_box_1
  - green_box_1

# Specify the object primitive (box/cylinder/sphere)
object_types:
  - "box"
  - "box"

# For each ID in `object_ids`, specify the following data
blue_box_1: {
  l: 0.064, # primitive dimensions
  w: 0.044,
  h: 0.16,
}

green_box_1: {
  l: 0.064,
  w: 0.044,
  h: 0.16,
}

# Corresponding to each ID, apply a simple orientation if you would like, the default is `up_x`
object_orientation_types:
  - up_y
  - up_y

# Specify the initial locations of the objects
initial_locations: {
  blue_box_1: "L1",
  green_box_1: "L2",
}
```

This file is similar to `workspace.yaml`, however it differs in a few ways.
1. You may omit the `(x,y,z)` coordinate from the data. If `x`, `y`, and `z` are specified, the object will spawn in that position.
2. The `object_orientation_types` field additionally specifies how the object will be grabbed. By default, the object will be grabbed along the **length**. In this example, we would like the object to be grabbed along the width, so we use an orientation type of `up_y` (yaw 90).
3. The `initial_locations` field can be used to easily initialize the object location to one of the discrete locations defined in `environment.yaml`. This feature is particularly useful for simulation experiments. This field is **optional**, however, if both the `initial_locations` field and `(x,y,z)` coordinate are ommitted, the object with spawn at the origin.

When using a pose tracker, you do not need to worry about specifying `(x,y,z)` or an initial location, as the pose information will be updated upon launch of the `manipulator_node`.

## Building a node that calls built-in action primitives (Intermediate)
We will build a simple C++ ROS node that will call a few of the built-in action primitives in TaskIt. 

TODO

## Making your own Action Primitive (Advanced)
We will now look at how to create your own action primitive using the TaskIt C++ library.

TODO