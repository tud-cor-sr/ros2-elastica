import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np
# Import Wrappers 
from elastica.wrappers import BaseSystemCollection, Constraints, Forcing #elastica.wrappers classes make it easy to construct different simulation systems
# Import Cosserat Rod Class
from elastica.rod.cosserat_rod import CosseratRod
# Import Boundary Condition Classes
from elastica.boundary_conditions import OneEndFixedRod, FreeRod
from elastica.external_forces import EndpointForces
# Import Timestepping Functions
from elastica.timestepper.symplectic_steppers import PositionVerlet
from elastica.timestepper import integrate

class MinimalPublisher(Node):
    
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello Bitch: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()



#Now create our beam system by combining all the wrappers we need to represent the physics that we to include in the simulation

class TimoshenkoBeamSimulator(BaseSystemCollection, Constraints, Forcing):
    pass

timoshenko_sim = TimoshenkoBeamSimulator()

# All of the values defined here are done in SI units, though this is not strictly necessary.

# setting up test params
n_elem = 100

density = 1000
nu = 0.1
E = 1e6
# For shear modulus of 1e4, nu is 99!
poisson_ratio = 99 # here for demonstration purposes (Poisson ratios can not exceed 0.5)


start = np.zeros((3,))
direction = np.array([0.0, 0.0, 1.0])
normal = np.array([0.0, 1.0, 0.0])
base_length = 3.0
base_radius = 0.25
base_area = np.pi * base_radius ** 2

# Important: Make sure that any rods you create get added to the simulator system (timoshenko_sim), otherwise they will not be included in your simulation.

shearable_rod = CosseratRod.straight_rod(
    n_elem,
    start,
    direction,
    normal,
    base_length,
    base_radius,
    density,
    nu,
    E,
    poisson_ratio,
)

timoshenko_sim.append(shearable_rod)

# With the rod added to the system, we need to apply boundary conditions

timoshenko_sim.constrain(shearable_rod).using(
    OneEndFixedRod, 
    constrained_position_idx=(0,), 
    constrained_director_idx=(0,)
)
print('One end of the rod is now fixed in place')

origin_force = np.array([0.0, 0.0, 0.0])
end_force = np.array([-10.0, 0.0, 0.0])
ramp_up_time = 5.0

timoshenko_sim.add_forcing_to(shearable_rod).using(
    EndpointForces, 
    origin_force, 
    end_force, 
    ramp_up_time=ramp_up_time
)
print('Forces added to the rod')

# Start into the plane
unshearable_start = np.array([0.0, -1.0, 0.0])
unshearable_rod = CosseratRod.straight_rod(
    n_elem,
    unshearable_start,
    direction,
    normal,
    base_length,
    base_radius,
    density,
    nu,
    E,
    # Unshearable rod needs G -> inf, which is achievable with a poisson ratio of -1.0
    poisson_ratio=-0.85,
)

timoshenko_sim.append(unshearable_rod)
timoshenko_sim.constrain(unshearable_rod).using(
    OneEndFixedRod, constrained_position_idx=(0,), constrained_director_idx=(0,)
)

timoshenko_sim.add_forcing_to(unshearable_rod).using(
    EndpointForces, origin_force, end_force, ramp_up_time=ramp_up_time
)
print('Unshearable rod set up')

#System finalization

#Note: if you make any changes to the rod after calling finalize, you will need to re-setup the system. This requires rerunning the code above this point.
timoshenko_sim.finalize()
print('System finalized')

#Simulation Time
final_time = 30.0
dl = base_length / n_elem
dt = 0.01 * dl
total_steps = int(final_time / dt)
print("Total steps to take", total_steps)

timestepper = PositionVerlet()

#Run Simulation
integrate(timestepper, timoshenko_sim, final_time, total_steps)  # 1min to run


# Post processing results

# Compute beam position for sherable and unsherable beams. 
def analytical_result(arg_rod, arg_end_force, shearing=True, n_elem=500):
    base_length = np.sum(arg_rod.rest_lengths)
    arg_s = np.linspace(0.0, base_length, n_elem)
    if type(arg_end_force) is np.ndarray:
        acting_force = arg_end_force[np.nonzero(arg_end_force)]
    else:
        acting_force = arg_end_force
    acting_force = np.abs(acting_force)
    linear_prefactor    = -acting_force / arg_rod.shear_matrix[0, 0, 0]
    quadratic_prefactor = -acting_force / 2.0 * np.sum(arg_rod.rest_lengths / arg_rod.bend_matrix[0, 0, 0])
    cubic_prefactor     = (acting_force / 6.0) / arg_rod.bend_matrix[0, 0, 0] 
    if shearing:    
        return arg_s, arg_s*linear_prefactor + arg_s**2*quadratic_prefactor + arg_s**3*cubic_prefactor
    else:
        return arg_s, arg_s**2 * quadratic_prefactor + arg_s**3 * cubic_prefactor


def plot_timoshenko(shearable_rod, unshearable_rod, end_force):
    # import matplotlib
    # matplotlib.use('TKAgg')
    
    import matplotlib.pyplot as plt
    analytical_shearable_positon = analytical_result(shearable_rod, end_force, shearing=True)
    analytical_unshearable_positon = analytical_result(unshearable_rod, end_force, shearing=False)
    
    fig = plt.figure(figsize=(5, 4), frameon=True, dpi=150)
    ax = fig.add_subplot(111)
    ax.grid(b=True, which="major", color="grey", linestyle="-", linewidth = 0.25)

    ax.plot(analytical_shearable_positon[0],   analytical_shearable_positon[1],  "k--", label="Timoshenko")
    ax.plot(analytical_unshearable_positon[0], analytical_unshearable_positon[1],"k-.", label="Euler-Bernoulli")
    
    ax.plot(shearable_rod.position_collection[2, :], 
            shearable_rod.position_collection[0, :], 
            "b-", label="n="+str(shearable_rod.n_elems))
    ax.plot(unshearable_rod.position_collection[2, :], 
            unshearable_rod.position_collection[0, :],
            "r-", label="n="+str(unshearable_rod.n_elems))
    
    ax.legend(prop={"size": 12})
    ax.set_ylabel('Y Position (m)', fontsize = 12)
    ax.set_xlabel('X Position (m)', fontsize = 12)
    for i in range(10):
        plt.show()
        print(i)

plot_timoshenko(shearable_rod, unshearable_rod, end_force)

if __name__ == '__main__':
    main()


#Dynamic Plotting (inefficient way)

# time = 0.0

# class BeamSimulator(BaseSystemCollection, Constraints, Forcing):
#     pass
# dynamic_update_sim = BeamSimulator()

# shearable_rod_new = CosseratRod.straight_rod(
#     n_elem,start,direction,normal,base_length,base_radius,density,nu,E,poisson_ratio,)
# dynamic_update_sim.append(shearable_rod_new)
# dynamic_update_sim.constrain(shearable_rod_new).using(
#     OneEndFixedRod, constrained_position_idx=(0,), constrained_director_idx=(0,))
# dynamic_update_sim.add_forcing_to(shearable_rod_new).using(
#     EndpointForces, origin_force, end_force, ramp_up_time=ramp_up_time)

# unshearable_rod_new = CosseratRod.straight_rod(
#     n_elem,unshearable_start,direction,normal,base_length,base_radius,density,nu,E,poisson_ratio=-0.85,)
# dynamic_update_sim.append(unshearable_rod_new)
# dynamic_update_sim.constrain(unshearable_rod_new).using(
#     OneEndFixedRod, constrained_position_idx=(0,), constrained_director_idx=(0,))
# dynamic_update_sim.add_forcing_to(unshearable_rod_new).using(
#     EndpointForces, origin_force, end_force, ramp_up_time=ramp_up_time)

# dynamic_update_sim.finalize()

# def run_and_update_plot(simulator, dt, start_time, stop_time, ax):
#     from elastica.timestepper import extend_stepper_interface
#     from elastica.timestepper.symplectic_steppers import PositionVerlet
#     timestepper = PositionVerlet()
#     do_step, stages_and_updates = extend_stepper_interface(timestepper, simulator)

#     n_steps = int((stop_time - start_time)/dt)
#     time = start_time
#     for i in range(n_steps):
#         time = do_step(timestepper, stages_and_updates, simulator, time, dt)
#     plot_timoshenko_dynamic(shearable_rod_new, unshearable_rod_new, end_force, time, ax)
#     return time

# def plot_timoshenko_dynamic(shearable_rod, unshearable_rod, end_force, time, ax):
#     import matplotlib.pyplot as plt
#     from IPython import display
    
#     analytical_shearable_positon = analytical_result(shearable_rod, end_force, shearing=True)
#     analytical_unshearable_positon = analytical_result(unshearable_rod, end_force, shearing=False)
    
#     ax.clear()
#     ax.grid(b=True, which="major", color="grey", linestyle="-", linewidth = 0.25)
#     ax.plot(analytical_shearable_positon[0],   analytical_shearable_positon[1],  "k--", label="Timoshenko")
#     ax.plot(analytical_unshearable_positon[0], analytical_unshearable_positon[1],"k-.", label="Euler-Bernoulli")
    
#     ax.plot(shearable_rod.position_collection[2, :], 
#             shearable_rod.position_collection[0, :], 
#             "b-", label="shearable rod")
#     ax.plot(unshearable_rod.position_collection[2, :], 
#             unshearable_rod.position_collection[0, :],
#             "r-", label="unshearable rod")
    
#     ax.legend(prop={"size": 12}, loc="lower left")
#     ax.set_ylabel('Y Position (m)', fontsize = 12)
#     ax.set_xlabel('X Position (m)', fontsize = 12)
#     ax.set_title('Simulation Time: %0.2f seconds' % time)
#     ax.set_xlim([-0.1, 3.1])
#     ax.set_ylim([-0.045, 0.002])


# %matplotlib inline
# import matplotlib.pyplot as plt
# from IPython import display

# evolve_for_time = 10.0
# update_interval = 1.0e-1

# # update the plot every 1 second
# fig = plt.figure(figsize=(5, 4), frameon=True, dpi=150)
# ax = fig.add_subplot(111)
# first_interval_time = update_interval + time
# last_interval_time = time + evolve_for_time 
# for stop_time in np.arange(first_interval_time, last_interval_time+dt, update_interval):
#     time = run_and_update_plot(dynamic_update_sim, dt, time, stop_time, ax)
#     display.clear_output(wait=True)
#     display.display(plt.gcf())
# plt.close()