# Custom message for the position, velocity, orientation of each rod of the soft robot arm

std_msgs/Header header 

# number of rods or segments of the robot
int64 num_rods

# all the rod states
RodState[] rod_states
