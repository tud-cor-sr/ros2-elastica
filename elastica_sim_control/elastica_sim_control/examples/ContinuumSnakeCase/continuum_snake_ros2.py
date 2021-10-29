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

rod_state = defaultdict(list)
rod_state["rod_tip_orientation"] = mp.Array('d', 4) #Quaternion form

# The array is of size 51 according to 'number of elements for Cosserat rod + 1'
rod_state["position_x"] = mp.Array('d',51)    
rod_state["position_y"] = mp.Array('d',51)
rod_state["position_z"] = mp.Array('d',51)
rod_state["velocity_x"] = mp.Array('d',51)
rod_state["velocity_y"] = mp.Array('d',51)
rod_state["velocity_z"] = mp.Array('d',51)
print_params = int(input("Enter '1' for printing the subscriber callback parameters' values and '0' for otherwise \n"))


class SnakeSimulator(BaseSystemCollection, Constraints, Forcing, CallBacks):
    pass



    
def run_snake(
    b_coeff, PLOT_FIGURE=False, SAVE_FIGURE=False, SAVE_VIDEO=False, SAVE_RESULTS=False
):
    
    sim_params = defaultdict(list)
    
    sim_params["b_coeff"] = b_coeff
    sim_params["t_coeff_optimized"] = b_coeff

    snake_sim = SnakeSimulator()
    

    # setting up test params
    sim_params["n_elem"] = 50
    sim_params["start"] = np.zeros((3,))
    sim_params["direction_MuscleTorques"] = np.array([0.0, 0.0, 1.0])
    sim_params["normal"] = np.array([0.0, 1.0, 0.0])
    sim_params["base_length"] = 1.0
    sim_params["base_radius"] = 0.025
    sim_params["density"] = 1000
    sim_params["nu"] = 5.0
    sim_params["E"] = 1e7
    sim_params["poisson_ratio"] = 0.5
    
    


    shearable_rod = CosseratRod.straight_rod(
        sim_params["n_elem"],
        sim_params["start"],
        sim_params["direction_MuscleTorques"],
        sim_params["normal"],
        sim_params["base_length"],
        sim_params["base_radius"],
        sim_params["density"],
        sim_params["nu"],
        sim_params["E"],
        sim_params["poisson_ratio"],
    )

    snake_sim.append(shearable_rod)

    # Add gravitational forces
    sim_params["gravitational_acc"] = -9.80665
    snake_sim.add_forcing_to(shearable_rod).using(
        GravityForces, acc_gravity=np.array([0.0, sim_params["gravitational_acc"], 0.0])
    )
    sim_params["period"] = 1.0
    sim_params["wave_length"] = sim_params["b_coeff"][-1]
    sim_params["wave_number"]=2.0 * np.pi / (sim_params["wave_length"])
    sim_params["phase_shift"]=0.0
    sim_params["rest_lengths"]=shearable_rod.rest_lengths
    sim_params["ramp_up_time_MuscleTorques"]=sim_params["period"]
    sim_params["with_spline"]=True
    sim_params["my_spline"] = np.ones(np.cumsum(shearable_rod.rest_lengths).shape)
    sim_params["time"] = 1.0
    sim_params["angular_frequency"] = 2.0 * np.pi / sim_params["period"]
    sim_params["factor"] = min(1.0, sim_params["time"] / sim_params["period"])
    sim_params["force"] = 0.0
    sim_params["direction_UniformForces"] = np.array([0.0, 0.0, 0.0])
    sim_params["torque"] = 0.0
    sim_params["direction_UniformTorques"] = np.array([0.0, 0.0, 0.0]) 
    sim_params["start_force"] =  np.array([0.0, 0.0, 0.0])
    sim_params["end_force"] =  np.array([0.0, 0.0, 0.0])
    sim_params["ramp_up_time_EndpointForces"] = 0.0
    sim_params["acc_gravity"] =  np.array([0.0, 0.0, 0.0])
    sim_params["dynamic_viscosity"] = 0.0 
    sim_params["start_force_mag"] =  0.0
    sim_params["end_force_mag"] =  0.0
    sim_params["ramp_up_time_EndpointForcesSinusoidal"] = 0.0
    sim_params["tangent_direction"] = np.array([0.0, 0.0, 0.0])
    sim_params["normal_direction"] = np.array([0.0, 0.0, 0.0])
    
    sim_params["muscle_torque_mag"] = muscle_torque_mag_cal(sim_params["factor"], sim_params["my_spline"], sim_params["angular_frequency"], sim_params["time"], sim_params["wave_length"], sim_params["rest_lengths"])
    sim_params["uniformforces_mag"] = uniformforces_mag_cal(sim_params["force"],sim_params["direction_UniformForces"], sim_params["n_elem"])
    sim_params["uniformtorques_mag"] = uniformtorques_mag_cal(sim_params["torque"], sim_params["direction_UniformTorques"], sim_params["n_elem"])
    

   
    
    
    snake_sim.add_forcing_to(shearable_rod).using(
        MuscleTorques,
        base_length= sim_params["base_length"] ,
        b_coeff=sim_params["b_coeff"][:-1],
        period=sim_params["period"],
        wave_number = sim_params["wave_number"],
        phase_shift=sim_params["phase_shift"],
        rest_lengths= sim_params["rest_lengths"],
        ramp_up_time=sim_params["ramp_up_time_MuscleTorques"],
        direction=sim_params["direction_MuscleTorques"],
        with_spline=sim_params["with_spline"],
    )
    
    
    # Add friction forces
    sim_params["origin_plane"] = np.array([0.0, -sim_params["base_radius"], 0.0])
    sim_params["normal_plane"] = sim_params["normal"]
    sim_params["slip_velocity_tol"] = 1e-8
    sim_params["froude"] = 0.1
    sim_params["mu"] = sim_params["base_length"] / (sim_params["period"] * sim_params["period"] * np.abs(sim_params["gravitational_acc"]) * sim_params["froude"])
    kinetic_mu_array = np.array(
        [sim_params["mu"], 1.5 * sim_params["mu"], 2.0 * sim_params["mu"]]
    )  # [forward, backward, sideways]
    static_mu_array = 2 * kinetic_mu_array
    snake_sim.add_forcing_to(shearable_rod).using(
        AnisotropicFrictionalPlane,
        k=1.0,
        nu=1e-6,
        plane_origin=sim_params["origin_plane"],
        plane_normal=sim_params["normal_plane"],
        slip_velocity_tol=sim_params["slip_velocity_tol"],
        static_mu_array=static_mu_array,
        kinetic_mu_array=kinetic_mu_array,
    )

    final_time = (11.0 + 0.01) * sim_params["period"]
    dt = 1.0e-3 * sim_params["period"]
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
                rod_state["rod_tip_orientation"][:] = [qw, qx, qy, qz]
                rod_state["position_x"][:] = system.position_collection[0]
                rod_state["position_y"][:] = system.position_collection[1]
                rod_state["position_z"][:] = system.position_collection[2]
                rod_state["velocity_x"][:] = system.velocity_collection[0]
                rod_state["velocity_y"][:] = system.velocity_collection[1]
                rod_state["velocity_z"][:] = system.velocity_collection[2]

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
        
        minimal_publisher_subcriber = MinimalPublisherSubscriberForces(sim_params, rod_state,print_params)
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
        plot_snake_velocity(pp_list, sim_params["period"], filename_plot, SAVE_FIGURE)

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
    [_, _, avg_forward, avg_lateral] = compute_projected_velocity(pp_list, sim_params["period"])

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