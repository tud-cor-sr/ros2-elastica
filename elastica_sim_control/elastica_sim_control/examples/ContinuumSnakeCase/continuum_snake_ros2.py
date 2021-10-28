__doc__ = """Continuum snake example, for detailed explanation refer to Gazzola et. al. R. Soc. 2018  
section 5.2 """

import numpy as np
import sys
import rclpy
import multiprocessing as mp
import time as T
import pickle

# FIXME without appending sys.path make it more generic
sys.path.append("../../")
import os
from elastica import *
from examples.ContinuumSnakeCase.continuum_snake_postprocessing import (
    plot_snake_velocity,
    plot_video,
    compute_projected_velocity,
)

sys.path.append("../../ros2_elastica_torque_force_params")
sys.path.append("../../utils")

from ros2_elastica_torque_force_params import *
from utils import *

rod_tip_orientation = mp.Array('d', 4) #Quaternion form

# The array is of size 51 according to 'number of elements for Cosserat rod + 1'
position_x = mp.Array('d',51)    
position_y = mp.Array('d',51)
position_z = mp.Array('d',51)
velocity_x = mp.Array('d',51)
velocity_y = mp.Array('d',51)
velocity_z = mp.Array('d',51)
print_params = int(input("Enter '1' for printing the subscriber callback parameters' values and '0' for otherwise \n"))


class SnakeSimulator(BaseSystemCollection, Constraints, Forcing, CallBacks):
    pass



    
def run_snake(
    b_coeff, PLOT_FIGURE=False, SAVE_FIGURE=False, SAVE_VIDEO=False, SAVE_RESULTS=False
):
    
    t_coeff_optimized = b_coeff

    snake_sim = SnakeSimulator()
    

    # setting up test params
    n_elem = 50
    start = np.zeros((3,))
    direction_MuscleTorques = np.array([0.0, 0.0, 1.0])
    normal = np.array([0.0, 1.0, 0.0])
    base_length = 1.0
    base_radius = 0.025
    density = 1000
    nu = 5.0
    E = 1e7
    poisson_ratio = 0.5
    
    


    shearable_rod = CosseratRod.straight_rod(
        n_elem,
        start,
        direction_MuscleTorques,
        normal,
        base_length,
        base_radius,
        density,
        nu,
        E,
        poisson_ratio,
    )

    snake_sim.append(shearable_rod)

    # Add gravitational forces
    gravitational_acc = -9.80665
    snake_sim.add_forcing_to(shearable_rod).using(
        GravityForces, acc_gravity=np.array([0.0, gravitational_acc, 0.0])
    )
    period = 1.0
    wave_length = b_coeff[-1]
    wave_number=2.0 * np.pi / (wave_length)
    phase_shift=0.0
    rest_lengths=shearable_rod.rest_lengths
    ramp_up_time_MuscleTorques=period
    with_spline=True
    my_spline = np.ones(np.cumsum(shearable_rod.rest_lengths).shape)
    time = 1.0
    angular_frequency = 2.0 * np.pi / period
    factor = min(1.0, time / period)
    force = 0.0
    direction_UniformForces = np.array([0.0, 0.0, 0.0])
    torque = 0.0
    direction_UniformTorques = np.array([0.0, 0.0, 0.0]) 
    start_force =  np.array([0.0, 0.0, 0.0])
    end_force =  np.array([0.0, 0.0, 0.0])
    ramp_up_time_EndpointForces = 0.0
    acc_gravity =  np.array([0.0, 0.0, 0.0])
    dynamic_viscosity = 0.0 
    start_force_mag =  0.0
    end_force_mag =  0.0
    ramp_up_time_EndpointForcesSinusoidal = 0.0
    tangent_direction = np.array([0.0, 0.0, 0.0])
    normal_direction = np.array([0.0, 0.0, 0.0])
    
    muscle_torque_mag = muscle_torque_mag_cal(factor, my_spline, angular_frequency, time, wave_length, rest_lengths)
    uniformforces_mag = uniformforces_mag_cal(force,direction_UniformForces, n_elem)
    uniformtorques_mag = uniformtorques_mag_cal(torque, direction_UniformTorques, n_elem)

   
    
    
    snake_sim.add_forcing_to(shearable_rod).using(
        MuscleTorques,
        base_length= base_length ,
        b_coeff=b_coeff[:-1],
        period=period,
        wave_number = wave_number,
        phase_shift=phase_shift,
        rest_lengths= rest_lengths,
        ramp_up_time=ramp_up_time_MuscleTorques,
        direction=direction_MuscleTorques,
        with_spline=with_spline,
    )
    
    
    # Add friction forces
    origin_plane = np.array([0.0, -base_radius, 0.0])
    normal_plane = normal
    slip_velocity_tol = 1e-8
    froude = 0.1
    mu = base_length / (period * period * np.abs(gravitational_acc) * froude)
    kinetic_mu_array = np.array(
        [mu, 1.5 * mu, 2.0 * mu]
    )  # [forward, backward, sideways]
    static_mu_array = 2 * kinetic_mu_array
    snake_sim.add_forcing_to(shearable_rod).using(
        AnisotropicFrictionalPlane,
        k=1.0,
        nu=1e-6,
        plane_origin=origin_plane,
        plane_normal=normal_plane,
        slip_velocity_tol=slip_velocity_tol,
        static_mu_array=static_mu_array,
        kinetic_mu_array=kinetic_mu_array,
    )

    final_time = (11.0 + 0.01) * period
    dt = 1.0e-3 * period
    total_steps = int(final_time / dt)

    # Add call backs
    class ContinuumSnakeCallBack(CallBackBaseClass):
        """
        Call back function for continuum snake
        """

        def __init__(self, step_skip: int, callback_params: dict):
            CallBackBaseClass.__init__(self)
            self.every = step_skip
            self.callback_params = callback_params
            self.pp_list_copy = defaultdict(list)

        def make_callback(self, system, time, current_step: int):

            if current_step % self.every == 0:
                Q = system.director_collection[..., -1]
                qw = np.sqrt(1 + Q[0, 0] + Q[1, 1] + Q[2, 2]) / 2
                qx = (Q[2, 1] - Q[1, 2]) / (4 * qw)
                qy = (Q[0, 2] - Q[2, 0]) / (4 * qw)
                qz = (Q[1, 0] - Q[0, 1]) / (4 * qw)
                rod_tip_orientation[:] = [qw, qx, qy, qz]
                position_x[:] = system.position_collection[0]
                position_y[:] = system.position_collection[1]
                position_z[:] = system.position_collection[2]
                velocity_x[:] = system.velocity_collection[0]
                velocity_y[:] = system.velocity_collection[1]
                velocity_z[:] = system.velocity_collection[2]

                if current_step ==total_steps:
                    pp_list_file = open("continuum_flagella.dat", "wb")
                    pickle.dump(self.pp_list_copy, pp_list_file)
                    pp_list_file.close()

                self.pp_list_copy["time"].append(time)
                self.pp_list_copy["step"].append(current_step)
                self.pp_list_copy["position"].append(
                    system.position_collection.copy()
                )
                self.pp_list_copy["velocity"].append(
                    system.velocity_collection.copy()
                )
                self.pp_list_copy["avg_velocity"].append(
                    system.compute_velocity_center_of_mass()
                )
                self.pp_list_copy["center_of_mass"].append(
                    system.compute_position_center_of_mass()
                )

                return

    pp_list = defaultdict(list)

    
    snake_sim.collect_diagnostics(shearable_rod).using(
        ContinuumSnakeCallBack, step_skip=200, callback_params=pp_list
    )

    snake_sim.finalize()
    timestepper = PositionVerlet()
    # timestepper = PEFRL()

    
    print("Total steps", total_steps)

    def ros_node():
        rclpy.init(args=None)
        
        minimal_publisher_subcriber = MinimalPublisherSubscriberForces(t_coeff_optimized ,period, wave_length, base_length, wave_number,phase_shift, 
                                    rest_lengths, ramp_up_time_MuscleTorques,direction_MuscleTorques, with_spline, muscle_torque_mag, force, 
                                    direction_UniformForces, uniformforces_mag, torque, direction_UniformTorques, uniformtorques_mag, start_force,
                                    end_force, ramp_up_time_EndpointForces, acc_gravity, dynamic_viscosity, start_force_mag, end_force_mag, 
                                    ramp_up_time_EndpointForcesSinusoidal, tangent_direction, normal_direction, rod_tip_orientation, position_x, 
                                    position_y, position_z, velocity_x,velocity_y,velocity_z,print_params)
        rclpy.spin(minimal_publisher_subcriber)

        

    p1 = mp.Process(target=ros_node)
    p2 = mp.Process(target=integrate, args = (timestepper, snake_sim, final_time, total_steps,))

    # starting process 1
    p1.start()
    # starting process 2
    p2.start()
    
    # wait until process 2 is finished
    p2.join()
    if not p2.is_alive():
        p1.terminate()

    pp_list_file = open("continuum_snake.dat", "rb")
    pp_list = pickle.load(pp_list_file)

    

    if PLOT_FIGURE:
        filename_plot = "continuum_snake_velocity.png"
        plot_snake_velocity(pp_list, period, filename_plot, SAVE_FIGURE)

        if SAVE_VIDEO:
            filename_video = "continuum_snake.mp4"
            plot_video(pp_list, video_name=filename_video, margin=0.2, fps=500)

    if SAVE_RESULTS:

        pass
    
    else:
        if os.path.exists("continuum_snake.dat"):
            os.remove("continuum_snake.dat")
        else:
            print("The file does not exist")

    # Compute the average forward velocity. These will be used for optimization.
    [_, _, avg_forward, avg_lateral] = compute_projected_velocity(pp_list, period)

    return avg_forward, avg_lateral, pp_list

def main():

    # Options
    PLOT_FIGURE = True
    SAVE_FIGURE = False
    SAVE_VIDEO = False
    SAVE_RESULTS = False
    CMA_OPTION = False

    if CMA_OPTION:

        import cma

        SAVE_OPTIMIZED_COEFFICIENTS = False

        def optimize_snake(spline_coefficient):
            [avg_forward, _, _] = run_snake(
                spline_coefficient,
                PLOT_FIGURE=False,
                SAVE_FIGURE=False,
                SAVE_VIDEO=False,
                SAVE_RESULTS=False,
            )
            # print("Optimizing snake")
            return -avg_forward

        # Optimize snake for forward velocity. In cma.fmin first input is function
        # to be optimized, second input is initial guess for coefficients you are optimizing
        # for and third input is standard deviation you initially set.
        optimized_spline_coefficients = cma.fmin(optimize_snake, 5 * [0], 0.5)

        # Save the optimized coefficients to a file
        filename_data = "optimized_coefficients.txt"
        if SAVE_OPTIMIZED_COEFFICIENTS:
            assert filename_data != "", "provide a file name for coefficients"
            np.savetxt(filename_data, optimized_spline_coefficients, delimiter=",")

    else:
        # Add muscle forces on the rod
        if os.path.exists("optimized_coefficients.txt"):
            t_coeff_optimized = np.genfromtxt(
                "optimized_coefficients.txt", delimiter=","
            )
            wave_length = 0.97 * 1.0  # 1.0 is base length
            t_coeff_optimized = np.hstack((t_coeff_optimized, wave_length))

        else:
            t_coeff_optimized = np.array([17.4, 48.5, 5.4, 14.7, 0.97])

        # run the simulation
        
        [avg_forward, avg_lateral, pp_list] = run_snake(
            t_coeff_optimized, PLOT_FIGURE, SAVE_FIGURE, SAVE_VIDEO, SAVE_RESULTS
        )

        print("average forward velocity:", avg_forward)
        print("average forward lateral:", avg_lateral)
        
        
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher_subcriber.destroy_node()
        
        
        
    # rclpy.shutdown()






    

if __name__ == "__main__":
    
    main()