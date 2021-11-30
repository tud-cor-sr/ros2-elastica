__doc__ = """Continuum snake example, for detailed explanation refer to Gazzola et. al. R. Soc. 2018  
section 5.2 """

import numpy as np
import rclpy
import multiprocessing as mp
import time as T
import pickle
from elastica.timestepper import extend_stepper_interface
from elastica._calculus import _isnan_check
from tqdm import tqdm

# FIXME without appending sys.path make it more generic
# sys.path.append("../../")
import os
from elastica import *
from elastica_sim_control.continuum_snake_postprocessing import (
    plot_snake_velocity,
    plot_video,
    compute_projected_velocity,
)

#you can choose a different number of control points or torque scaling factor by changing the number_of_control_points and alpha
from elastica_sim_control import MuscleTorquesWithVaryingBetaSplines


# sys.path.append("../../elastica_publisher_subscriber")
# sys.path.append("../../utils")

from elastica_sim_control.elastica_publisher_subscriber import *
from elastica_sim_control.utils import *

rod_state = defaultdict(list)

control_input = defaultdict(list)
control_input["control_points"]  = mp.Array('d', 18)

rod_state["rod_tip_orientation"] = mp.Array('d', 4) #Quaternion form

# The array is of size 51 according to 'number of elements for Cosserat rod + 1'
rod_state["position_x"] = mp.Array('d',51)    
rod_state["position_y"] = mp.Array('d',51)
rod_state["position_z"] = mp.Array('d',51)
rod_state["velocity_x"] = mp.Array('d',51)
rod_state["velocity_y"] = mp.Array('d',51)
rod_state["velocity_z"] = mp.Array('d',51)


class SnakeSimulator(BaseSystemCollection, Constraints, Forcing, CallBacks):
    pass



class DefineSnake():
    def __init__(self,b_coeff):
        self.sim_params = defaultdict(list)
        self.StatefulStepper = PositionVerlet()
        self.sim_params["b_coeff"] = b_coeff
        self.sim_params["period"] = 1.0
        self.sim_params["number_of_control_points"] = 6
        self.sim_params["alpha"] = 75
        
        self.sim_params["final_time"] = (10.0 + 0.01) * self.sim_params["period"]
        self.sim_params["dt"] = 2.5e-5 * self.sim_params["period"]
        self.sim_params["total_steps"] = int(self.sim_params["final_time"] / self.sim_params["dt"])
        
        rendering_fps = 60
        self.sim_params["time_step"] = np.float64(float(self.sim_params["final_time"]) / self.sim_params["total_steps"])
        self.sim_params["step_skip"] = int(1.0 / (rendering_fps * self.sim_params["time_step"]))
        self.sim_params["max_rate_of_change_of_activation"] = np.infty
        
        self.snake_sim = SnakeSimulator()
    
        # setting up test params
        self.sim_params["n_elem"] = 50
        self.sim_params["start"] = np.zeros((3,))
        self.sim_params["direction_of_rod_extension"] = np.array([0.0, 0.0, 1.0])
        self.sim_params["normal"] = np.array([0.0, 1.0, 0.0])
        self.sim_params["base_length"] = 1.0
        self.sim_params["base_radius"] = 0.025
        self.sim_params["density"] = 1000
        self.sim_params["nu"] = 5.0
        self.sim_params["E"] = 1e7
        self.sim_params["poisson_ratio"] = 0.5
        
        


        self.shearable_rod = CosseratRod.straight_rod(
            self.sim_params["n_elem"],
            self.sim_params["start"],
            self.sim_params["direction_of_rod_extension"],
            self.sim_params["normal"],
            self.sim_params["base_length"],
            self.sim_params["base_radius"],
            self.sim_params["density"],
            self.sim_params["nu"],
            self.sim_params["E"],
            self.sim_params["poisson_ratio"],
        )

        self.snake_sim.append(self.shearable_rod)

        # Add gravitational forces
        self.sim_params["gravitational_acc"] = -9.80665
        self.snake_sim.add_forcing_to(self.shearable_rod).using(
            GravityForces, acc_gravity=np.array([0.0, self.sim_params["gravitational_acc"], 0.0])
        )
        
        self.torque_profile_list_for_muscle_in_normal_dir = defaultdict(list)
        self.control_points_array_normal_dir = []
        # Apply torques
        self.snake_sim.add_forcing_to(self.shearable_rod).using(
            MuscleTorquesWithVaryingBetaSplines,
            base_length=self.sim_params["base_length"],
            number_of_control_points=self.sim_params["number_of_control_points"],
            points_func_array=self.control_points_array_normal_dir,
            muscle_torque_scale=self.sim_params["alpha"],
            direction=str("normal"),
            step_skip=self.sim_params["step_skip"],
            max_rate_of_change_of_activation=self.sim_params["max_rate_of_change_of_activation"],
            torque_profile_recorder=self.torque_profile_list_for_muscle_in_normal_dir,
        )
        
        
        self.sim_params["wave_length"] = self.sim_params["b_coeff"][-1]
        self.sim_params["wave_number"]=2.0 * np.pi / (self.sim_params["wave_length"])
        self.sim_params["phase_shift"]=0.0
        self.sim_params["rest_lengths"]=self.shearable_rod.rest_lengths
        self.sim_params["ramp_up_time_MuscleTorques"]=self.sim_params["period"]
        self.sim_params["with_spline"]=True
        self.sim_params["my_spline"] = np.ones(np.cumsum(self.shearable_rod.rest_lengths).shape)
        self.sim_params["time"] = 1.0
        self.sim_params["angular_frequency"] = 2.0 * np.pi / self.sim_params["period"]
        self.sim_params["factor"] = min(1.0, self.sim_params["time"] / self.sim_params["period"])
        self.sim_params["force"] = 0.0
        self.sim_params["direction_UniformForces"] = np.array([0.0, 0.0, 0.0])
        self.sim_params["torque"] = 0.0
        self.sim_params["direction_UniformTorques"] = np.array([0.0, 0.0, 0.0]) 
        self.sim_params["start_force"] =  np.array([0.0, 0.0, 0.0])
        self.sim_params["end_force"] =  np.array([0.0, 0.0, 0.0])
        self.sim_params["ramp_up_time_EndpointForces"] = 0.0
        self.sim_params["acc_gravity"] =  np.array([0.0, 0.0, 0.0])
        self.sim_params["dynamic_viscosity"] = 0.0 
        self.sim_params["start_force_mag"] =  0.0
        self.sim_params["end_force_mag"] =  0.0
        self.sim_params["ramp_up_time_EndpointForcesSinusoidal"] = 0.0
        self.sim_params["tangent_direction"] = np.array([0.0, 0.0, 0.0])
        self.sim_params["normal_direction"] = np.array([0.0, 0.0, 0.0])
        
        self.sim_params["muscle_torque_mag"] = muscle_torque_mag_cal(self.sim_params["factor"], self.sim_params["my_spline"], self.sim_params["angular_frequency"], self.sim_params["time"], self.sim_params["wave_length"], self.sim_params["rest_lengths"])
        self.sim_params["uniformforces_mag"] = uniformforces_mag_cal(self.sim_params["force"],self.sim_params["direction_UniformForces"], self.sim_params["n_elem"])
        self.sim_params["uniformtorques_mag"] = uniformtorques_mag_cal(self.sim_params["torque"], self.sim_params["direction_UniformTorques"], self.sim_params["n_elem"])
        
        
        # Add friction forces
        self.sim_params["origin_plane"] = np.array([0.0, -self.sim_params["base_radius"], 0.0])
        self.sim_params["normal_plane"] = self.sim_params["normal"]
        self.sim_params["slip_velocity_tol"] = 1e-8
        self.sim_params["froude"] = 0.1
        self.sim_params["mu"] = self.sim_params["base_length"] / (self.sim_params["period"] * self.sim_params["period"] * np.abs(self.sim_params["gravitational_acc"]) * self.sim_params["froude"])
        kinetic_mu_array = np.array(
            [self.sim_params["mu"], 1.5 * self.sim_params["mu"], 2.0 * self.sim_params["mu"]]
        )  # [forward, backward, sideways]
        static_mu_array = 2 * kinetic_mu_array
        self.snake_sim.add_forcing_to(self.shearable_rod).using(
            AnisotropicFrictionalPlane,
            k=1.0,
            nu=1e-6,
            plane_origin=self.sim_params["origin_plane"],
            plane_normal=self.sim_params["normal_plane"],
            slip_velocity_tol=self.sim_params["slip_velocity_tol"],
            static_mu_array=static_mu_array,
            kinetic_mu_array=kinetic_mu_array,
        )

        self.time_tracker = mp.Value('d', 0.0)
        
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

                    if time >= 10.0:
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

        self.pp_list = defaultdict(list)

        self.snake_sim.collect_diagnostics(self.shearable_rod).using(
            ContinuumSnakeCallBack, step_skip=200, callback_params=self.pp_list
        )

        self.snake_sim.finalize()
        # do_step, stages_and_updates will be used in step function
        self.do_step, self.stages_and_updates = extend_stepper_interface(
                                self.StatefulStepper,self.snake_sim
                            )
        
    
    def time_stepping(self, control_input):
        
        self.control_points_array_normal_dir[:] = control_input[
                : self.sim_params["number_of_control_points"]
            ]
        
        
        self.time_tracker.value = self.do_step(
            self.StatefulStepper,
            self.stages_and_updates,
            self.snake_sim,
            self.time_tracker.value,
            self.sim_params["time_step"],
            )
        done = False
        if self.time_tracker.value> 10.01:
            done =True
        
        # Checking for NaN positon of the rod. If so, stop the simulation
        invalid_values_condition = _isnan_check(self.shearable_rod.position_collection)

        if invalid_values_condition == True:
            print(" NaN detected, will exit the simulation")
            self.shearable_rod.position_collection = np.zeros(
                self.shearable_rod.position_collection.shape
            )
            done = True
        return done
    
control_input["control_points"][:] = ((np.random.rand(3 * 6) - 0.5) * 2)

    
def run_snake(
    b_coeff, PLOT_FIGURE=False, SAVE_FIGURE=False, SAVE_VIDEO=False, SAVE_RESULTS=False
):
    snake_run = DefineSnake(b_coeff)
    
    
    print("Total steps", snake_run.sim_params["total_steps"])
    
    def snake_stepping():
        done = False
        
        for i in tqdm(range(snake_run.sim_params["total_steps"])):
            done = snake_run.time_stepping(control_input["control_points"][:])
            if done:
                break
        
    
    def ros_node():
        rclpy.init(args=None)
        
        elastica_pub_sub = ElasticaPublisherSubscriber(snake_run.sim_params, rod_state,snake_run.time_tracker,control_input)
        rclpy.spin(elastica_pub_sub)

        
    #ROS2 Node
    p1 = mp.Process(target=ros_node)
    #Starting the simulation
    p2 = mp.Process(target=snake_stepping)

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
        plot_snake_velocity(pp_list, snake_run.sim_params["period"], filename_plot, SAVE_FIGURE)

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
    [_, _, avg_forward, avg_lateral] = compute_projected_velocity(pp_list, snake_run.sim_params["period"])

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