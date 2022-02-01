# ROS2 Elastica
Note: This is the ROS2 package involving python3 scripts for simulating the *Cosserat rods* in PyElastica with information being exchanged on ros topics. Down below are the instructions for running the whole package separately in a docker container which can virtually run in any environment, which make it easy to deploy & test this application. Also with instructions to build the package on a local env or in a different docker container other than the below.

## Instructions for pulling the sr-ros2-elastica image and running the container & python3 scripts

Here is the link to our [repository](https://github.com/tud-cor-sr/sr-ros2-bundles) containing ROS2 Docker images including Elastica ROS2 package for simulation of *Cosserat Rods*.
Clone the repo & follow the instructions mentioned in README to get the *sr-ros2-elastica* container running

Before running the below command, we have to allow client to connect to the server by running the following command in a separate terminal (it can be done after running the container as well but needed to be done before running the script where you want to use matplotlib or some other GUI based application inside the container):
```
    xhost +
```
When the above cloning, building & composing had taken you to the bash terminal then you can start executing the python scripts inside the container to simulate the cosserat rods. There are two scripts modified for simulating a 'continuum snake' & a 'continuum flagella' while publishing the parameters (for different torques & forces on each element of Cosserat rod simulated) on their respective ros topics & subscribing to the control input (control points for beta spline) throught spinning a ROS2 node.

They can be executed with the help of the following commands:

*Continuum Flagella Case (run with a launch file)*
```
    ros2 launch elastica_sim continuum_flagella_ros2.launch.py
```

*Continuum Snake Case (run with a launch file)*
```
    ros2 launch elastica_sim continuum_snake_ros2.launch.py
```

Note: In *continuum_snake_ros2.py* script, the average velocities' values sometimes goes to *NaN* due to wrong beta coefficient values which exits the simulation

To end the execution of the script, one can do so through Ctrl+C and Ctrl+D for closing & removing the running container after stopping the python script

*One can see the current ros topics being published with the help of the following command in a separate terminal:*

```
    sudo docker exec -it <CONTAINER> bash
    ros2 topic list
```

If one wants to run the scripts from their local system without a ros interface, kindly run the following scripts:

*Continuum Flagella Case*
```
    cd ~/path/to/ros2-elastica/elastica_sim/elastica_sim/examples/ContinuumFlagellaCase/
    python3 continuum_flagella.py 
```

*Continuum Snake Case*
```
    cd ~/path/to/ros2-elastica/elastica_sim/elastica_sim/examples/ContinuumSnakeCase/
    python3 continuum_snake.py 
```

## Instructions for building the ROS2 Elastica package (either on a docker container or a local env)

### Source & Return to the root of your workspace:
```
source /opt/ros/$ROS_DISTRO/setup.bash
cd /path/to/ros2_ws
```

### Build & source the workspace
```
colcon build --packages-select elastica_sim elastica_controllers
source install/local_setup.bash
```

### Run (same ros commands as above)

*Continuum Flagella Case (run with a launch file)*
```
    ros2 launch elastica_sim continuum_flagella_ros2.launch.py
```

*Continuum Snake Case (run with a launch file)*
```
    ros2 launch elastica_sim continuum_snake_ros2.launch.py
```

Here is the description of all the ros topics being published. Most of them are the parameters which decide the magnitude of the MuscleTorque, UniformTorques, UniformForces, EndPointForces, EndPointForcesSinusoidal which can be applied to each element of Cosserat rod simulated.

*Before giving the description first lets see the different types or classes of external forces which can be applied to the rod:*

```GravityForces``` This class applies a constant gravitational force to the entire rod.

```EndpointForces``` This class applies constant forces on the endpoint nodes.

```EndpointForcesSinusodial``` This class applies sinusodial forces on the endpoint nodes.

```UniformTorques``` This class applies a uniform torque to the entire rod.

```UniformForces``` This class applies a uniform force to the entire rod.

```MuscleTorques``` This class applies muscle torques along the body. The applied muscle torques are treated as applied external forces. This class can apply muscle torques as a traveling wave with a beta spline or only as a traveling wave.

```SlenderBodyTheory``` This slender body theory class is for flow-structure interaction problems. This class applies hydrodynamic forces on the body using the slender body theory given in Eq. 4.13 of Gazzola et al. RSoS (2018).

*So, lets continue with the description values in the ros topics namely /elastica/control_input, /elastica/time_tracker, /elastica/rods_state, /elastica/physical_params,  '/elastica/objs_state'*

```/acc_gravity``` array 1D (dim) array containing data with 'float' type. Gravitational acceleration vector. (*class GravityForces*) /elastica/physical_params

```/base_length``` Rest length of the rod-like object (float). (*class*) /elastica/physical_params

```/b_coeff``` optimized coefficients for a snake gait (*class MuscleTorques*) /elastica/physical_params

```/direction_of_rod_extension``` array 1D (dim) array containing data with 'float' type. direction in which rod extends (also the Muscle torque direction). /elastica/physical_params 

```/direction_UniformForces``` array1D (dim) array containing data with 'float' type. Direction in which force applied. (*class UniformForces*) /elastica/physical_params

```/direction_UniformTorques``` array 1D (dim) array containing data with 'float' type. Direction in which torque applied. (*class UniformTorques*) /elastica/physical_params

```/dynamic_viscosity``` Dynamic viscosity of the fluid (float) for slender body theory class (for flow-structure interaction problems). (*class SlenderBodyTheory*) /elastica/physical_params

```/elastica/control_input``` control input/points (varying) for uniform torque which determines the torque in various segments of continuum robot 

```/elastica/objs_state``` pose adn description of the objects in the environment

```/elastica/time_tracker``` Current simulation Time in seconds

```/end_force``` array 2D (dim, 1) array containing data with 'float' type. Force applied to last node of the rod-like object. (*class EndpointForces*) /elastica/physical_params

```/end_force_mag``` Magnitude of Force applied to last node of the rod-like object (*class EndpointForces*) /elastica/physical_params

```/force``` float Force magnitude applied to a rod-like object. (*class UniformForces*) /elastica/physical_params

```/muscle_torque_mag``` Magnitude of muscle torque getting applied on each element. (*class MuscleTorques*) /elastica/physical_params

```/normal_direction``` array 1D (dim) array containing data with 'float' type for applying force in normal direction. (*class EndpointForcesSinusodial*) /elastica/physical_params

```orientation_XX``` rod elements's orientation in quaternion form (last element of last segment being the tip of the continuum robot) /elastica/physical_params

```/parameter_events``` it provides a way to subscribe to all parameter updates occurring on the node, including addition removal and changes in value. Every atomic change will be published separately. 

```/period``` period of traveling wave in MuscleTorque class (float). (*class MuscleTorques*) /elastica/physical_params

```/phase_shift``` float, Phase shift of traveling wave. (*class MuscleTorques*) /elastica/physical_params

```position_x``` position of elements in direction of unit vector x /elastica/rods_state
 
```position_y``` position of elements in direction of unit vector y /elastica/rods_state

```position_z``` position of elements in direction of unit vector z /elastica/rods_state

```/ramp_up_time_EndpointForces``` float ,applied Endpoint Forces are ramped up until ramp up time. (*class EndpointForces*) /elastica/physical_params

```/ramp_up_time_EndpointForcesSinusoidal``` float ,applied Endpoint Sinusoidal Forces are ramped up until ramp up time. (*class EndpointForcesSinusodial*) /elastica/physical_params

```/ramp_up_time_MuscleTorques``` float ,applied muscle torques are ramped up until ramp up time. (*class MuscleTorques*) /elastica/physical_params

```/rest_lengths``` length of each element at rest. /elastica/physical_params

```/rosout``` ROS client libraries are required to publish console logging messages to the /rosout topic as a standard interface. 

```/start_force``` array 2D (dim, 1) array containing data with 'float' type. Force applied to first node of the rod-like object. (*class EndpointForces*) /elastica/physical_params

```/start_force_mag``` Magnitude of Force applied to first node of the rod-like object (*class EndpointForcesSinusodial*) /elastica/physical_params

```/tangent_direction``` array 1D (dim) array containing data with 'float' type for applying force in tangent direction (*class EndpointForcesSinusodial*) /elastica/physical_params

```/torque``` Torque magnitude applied to a rod-like object (float). (*class UniformTorques*) /elastica/physical_params

```/uniformforces_mag``` Magnitude of uniform forces getting applied on each element. (*class UniformForces*) /elastica/physical_params

```/uniformtorques_mag``` Magnitude of uniform torques getting applied on each element per segment (set of consecutive three elements in vector for each segment ). (*class UniformTorques*) /elastica/physical_params

```velocity_x``` velocity of elements in direction of unit vector x /elastica/rods_state

```velocity_y``` velocity of elements in direction of unit vector y /elastica/rods_state

```velocity_z``` velocity of elements in direction of unit vector z /elastica/rods_state

```/wave_length``` Wave length of traveling wave. (float) (*class MuscleTorques*) /elastica/physical_params

```/wave_number``` Wave number of traveling wave. (float) (*class MuscleTorques*) /elastica/physical_params

```/with_spline``` Option to use beta-spline. (boolean) (*class MuscleTorques*) /elastica/physical_params