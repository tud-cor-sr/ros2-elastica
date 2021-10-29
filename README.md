**Note:** This is the ROS2 package involving python3 scripts for simulating the *Cosserat rods* in PyElastica with information being exchanged on ros topics. Down below are the instructions for running the whole package separately in a docker container which can virtually run in any environment, which make it easy to deploy & test this application. Also with instructions to build the package on a local env or in a different docker container other than the below.

# Instructions for pulling the ros2_elastica image and running the container & python3 scripts

The current image works Linux kernel. First of all if you don't have the image then Pull the current distribution of Docker Image with the following command (all the development & testing is done on Ubuntu 18.04):
```
    sudo docker pull ruffy369/ros2_elastica:latest
```
Before running the below command, we have to allow client to connect to the server by running the following command in a separate terminal (it can be done after running the container as well but needed to be done before running the script where you want to use matplotlib or some other GUI based application inside the container):
```
    xhost +
```

After pulling the image, run an instance of the image (container) with the help of the following command:
```
    sudo docker run --rm -it  --name elastica_sim_control -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro  ruffy369/ros2_elastica:latest
```

**Breaking down the above command:**

*```-e DISPLAY=$DISPLAY```*  sends an environment variable to set the display of the server

*```-v /tmp/.X11-unix:/tmp/.X11-unix:ro```* mounts X11 socket inside the container in read-only mode (just to be on a safer side)

* When the above command had taken you to the bash terminal then you can start executing the python scripts inside the container to simulate the cosserat rods. There are two scripts modified for simulating a 'continuum snake' & a 'continuum flagella' while publishing the parameters (for different torques & forces on each element of Cosserat rod simulated) on their respective ros topics throught spinning a ROS2 node.

They can be executed with the help of the following commands:

*Continuum Flagella Case (run as a python script)*
```
    cd ~/ros2_elastica_ws/src/elastica_sim_control/elastica_sim_control/examples/ContinuumFlagellaCase/
    python3 continuum_flagella_ros2.py 
```
*Continuum Flagella Case (run as a executable ros2 node)*
```
    ros2 run elastica_sim_control continuum_flagella_ros2 

```
*Continuum Snake Case (run as a python script)*
```
    cd ~/ros2_elastica_ws/src/elastica_sim_control/elastica_sim_control/examples/ContinuumSnakeCase/
    python3 continuum_snake_ros2.py 
```
*Continuum Snake Case (run as a executable ros2 node)*
```
    ros2 run elastica_sim_control continuum_snake_ros2 

```
**Note:** *continuum_snake_ros2.py* script has wrong optimized coefficient values because of which the average velocities' values goes to *NaN*

To end the execution of the script, one can do so through Ctrl+C and Ctrl+D for closing & removing the running container after stopping the python script

*One can see the current ros topics being published with the help of the following command in a separate terminal:*

```
    sudo docker exec -it elastica_sim_control bash
    ros2 topic list
```

**If one wants to run the scripts from their local system without a ros interface, kindly run the following scripts which are provided by Pyelastica by default:**

*Continuum Flagella Case*
```
    cd ~/ros2_elastica_ws/src/elastica_sim_control/elastica_sim_control/examples/ContinuumFlagellaCase/
    python3 continuum_flagella.py 
```

*Continuum Snake Case*
```
    cd ~/ros2_elastica_ws/src/elastica_sim_control/elastica_sim_control/examples/ContinuumSnakeCase/
    python3 continuum_snake.py 
```

# Instructions for building the ROS2 package (either on a docker container or a local env)

## Source & Return to the root of your workspace:
```
source /opt/ros/$ROS_DISTRO/setup.bash
cd /path/to/ros2_ws
```

## Build & source the workspace
```
colcon build --packages-select elastica_sim_control
source install/local_setup.bash

```

## Run (same ros commands as above)

*Continuum Flagella Case*
```
    ros2 run elastica_sim_control continuum_flagella_ros2 

```

*Continuum Snake Case*
```
    ros2 run elastica_sim_control continuum_snake_ros2 

```

**Here is the description of all the ros topics being published. Most of them are the parameters which decide the magnitude of the MuscleTorque, UniformTorques, UniformForces, EndPointForces, EndPointForcesSinusoidal which can be applied to each element of Cosserat rod simulated.**

*Before giving the description first lets see the different types or classes of external forces which can be applied to the rod:*

```GravityForces``` This class applies a constant gravitational force to the entire rod.

```EndpointForces``` This class applies constant forces on the endpoint nodes.

```EndpointForcesSinusodial``` This class applies sinusodial forces on the endpoint nodes.

```UniformTorques``` This class applies a uniform torque to the entire rod.

```UniformForces``` This class applies a uniform force to the entire rod.

```MuscleTorques``` This class applies muscle torques along the body. The applied muscle torques are treated as applied external forces. This class can apply muscle torques as a traveling wave with a beta spline or only as a traveling wave.

```SlenderBodyTheory``` This slender body theory class is for flow-structure interaction problems. This class applies hydrodynamic forces on the body using the slender body theory given in Eq. 4.13 of Gazzola et al. RSoS (2018).

*So, lets continue with the description of the ros topics*

```/acc_gravity``` array 1D (dim) array containing data with 'float' type. Gravitational acceleration vector. (*class GravityForces*)

```/base_length``` Rest length of the rod-like object (float). (*class*)

```/direction_MuscleTorques``` array 1D (dim) array containing data with 'float' type. Muscle torque direction. (*class MuscleTorques*)

```/direction_UniformForces``` array1D (dim) array containing data with 'float' type. Direction in which force applied. (*class UniformForces*)

```/direction_UniformTorques``` array 1D (dim) array containing data with 'float' type. Direction in which torque applied. (*class UniformTorques*)

```/dynamic_viscosity``` Dynamic viscosity of the fluid (float) for slender body theory class (for flow-structure interaction problems). (*class SlenderBodyTheory*)

```/end_force``` array 2D (dim, 1) array containing data with 'float' type. Force applied to last node of the rod-like object. (*class EndpointForces*)

```/end_force_mag``` Magnitude of Force applied to last node of the rod-like object (*class EndpointForces*)

```/force``` float Force magnitude applied to a rod-like object. (*class UniformForces*)

```/muscle_torque_mag``` Magnitude of muscle torque getting applied on each element. (*class MuscleTorques*)

```/normal_direction``` array 1D (dim) array containing data with 'float' type for applying force in normal direction. (*class EndpointForcesSinusodial*)

```/parameter_events``` it provides a way to subscribe to all parameter updates occurring on the node, including addition removal and changes in value. Every atomic change will be published separately. 

```/period``` period of traveling wave in MuscleTorque class (float). (*class MuscleTorques*)

```/phase_shift``` float, Phase shift of traveling wave. (*class MuscleTorques*)

```position_x``` position of elements in direction of unit vector x

```position_y``` position of elements in direction of unit vector y

```position_z``` position of elements in direction of unit vector z

```/ramp_up_time_EndpointForces``` float ,applied Endpoint Forces are ramped up until ramp up time. (*class EndpointForces*)

```/ramp_up_time_EndpointForcesSinusoidal``` float ,applied Endpoint Sinusoidal Forces are ramped up until ramp up time. (*class EndpointForcesSinusodial*)

```/ramp_up_time_MuscleTorques``` float ,applied muscle torques are ramped up until ramp up time. (*class MuscleTorques*)

```/rest_lengths``` length of each element at rest. 

```/rod_tip_orientation``` rod tip's orientation in quaternion form 

```/rosout``` ROS client libraries are required to publish console logging messages to the /rosout topic as a standard interface. 

```/start_force``` array 2D (dim, 1) array containing data with 'float' type. Force applied to first node of the rod-like object. (*class EndpointForces*)

```/start_force_mag``` Magnitude of Force applied to first node of the rod-like object (*class EndpointForcesSinusodial*)

```/t_coeff_optimized``` optimized coefficients for a snake gait (*class MuscleTorques*)

```/tangent_direction``` array 1D (dim) array containing data with 'float' type for applying force in tangent direction (*class EndpointForcesSinusodial*)

```/torque``` Torque magnitude applied to a rod-like object (float). (*class UniformTorques*)

```/uniformforces_mag``` Magnitude of uniform forces getting applied on each element. (*class UniformForces*)

```/uniformtorques_mag``` Magnitude of uniform torques getting applied on each element. (*class UniformTorques*)

```velocity_x``` velocity of elements in direction of unit vector x

```velocity_y``` velocity of elements in direction of unit vector y

```velocity_z``` velocity of elements in direction of unit vector z

```/wave_length``` Wave length of traveling wave. (float) (*class MuscleTorques*)

```/wave_number``` Wave number of traveling wave. (float) (*class MuscleTorques*)

```/with_spline``` Option to use beta-spline. (boolean) (*class MuscleTorques*)