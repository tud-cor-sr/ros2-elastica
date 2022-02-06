# Custom message for the position, velocity, orientation of each element of the cosserat rod

std_msgs/Header header 

# number of elements per rod/segment
int64 num_elements

# poses of all elements in the Cosserat rod
geometry_msgs/PoseStamped[] poses
geometry_msgs/Vector3[] velocities

# curvature (kappa) of the Cosserat rod
geometry_msgs/Vector3[] kappa