bool execution_success

# Total time the action primitive took to execute
float64 execution_time

# Total joint-space path length for the given trajectory
float64 path_length

# Maximum velocity magnitude encountered along the planned trajectory across any joint
float64 max_velocity

# Maximum acceleration magnitude encountered along the planned trajectory across any joint
float64 max_acceleration

# Maximum effort magnitude encountered along the planned trajectory across any joint
float64 max_effort

# End-effector pose for each point along the trajectory
geometry_msgs/Pose[] eef_trajectory