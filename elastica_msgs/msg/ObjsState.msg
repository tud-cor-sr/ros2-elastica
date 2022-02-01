# Custom message for the position and orientation of single object simulated in the Elastica Environment

std_msgs/Header header 

# number of objects
int64 num_objects

#name/ids of objects
string obj_names

# poses of all the objects in the Environment
geometry_msgs/PoseStamped[] obj_poses

