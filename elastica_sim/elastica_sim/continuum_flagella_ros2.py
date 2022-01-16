__doc__ = """Continuum flagella example, for detailed explanation refer to Gazzola et. al. R. Soc. 2018
section 5.2.1 """

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
from elastica_sim.continuum_flagella_postprocessing import (
    plot_velocity,
    plot_video,
    compute_projected_velocity,
)


# sys.path.append("../../elastica_publisher_subscriber")
# sys.path.append("../../utils")

from elastica_sim.elastica_publisher_subscriber import *
from elastica_sim.utils import *

no_of_segments = 6 #Number of Pneumatic chambers in the Continuum robot arm
n_elements = 50 #number of elements for Cosserat rod
rod_state = [defaultdict(list)]*no_of_segments

control_input = defaultdict(list)

control_input["control_torque"]  = mp.Array('d', no_of_segments)
control_input["control_torque_dir"]  = mp.Array('d', no_of_segments*3)

def rod_state_mp_arr_create(n_segments):
    for i in range(n_segments):
        #Quaternion form (Orientaion of each element, last element of last segment being the Robot arm Tip)
        rod_state[i]["orientation_ww"] = mp.Array('d',n_elements) 
        rod_state[i]["orientation_xx"] = mp.Array('d',n_elements)
        rod_state[i]["orientation_yy"] = mp.Array('d',n_elements) 
        rod_state[i]["orientation_zz"] = mp.Array('d',n_elements)

        # The array is of size 51 according to 'number of elements for Cosserat rod + 1'
        rod_state[i]["position_x"] = mp.Array('d',n_elements+1)    
        rod_state[i]["position_y"] = mp.Array('d',n_elements+1)
        rod_state[i]["position_z"] = mp.Array('d',n_elements+1)
        rod_state[i]["velocity_x"] = mp.Array('d',n_elements+1)
        rod_state[i]["velocity_y"] = mp.Array('d',n_elements+1)
        rod_state[i]["velocity_z"] = mp.Array('d',n_elements+1)

rod_state_mp_arr_create(no_of_segments)

class FlagellaSimulator(BaseSystemCollection, Connections, Constraints, Forcing, CallBacks):
    pass

class DefineFlagella():
    def __init__(self,b_coeff):
        self.sim_params = defaultdict(list)
        
        self.StatefulStepper = PositionVerlet()
        self.sim_params["period"] = 1.0
        self.sim_params["b_coeff"] = b_coeff
        self.sim_params["wave_length"] = b_coeff[-1]
        self.sim_params["number_of_control_points"] = 6
        self.sim_params["alpha"] = 75
        
        self.sim_params["final_time"] = (10.0 + 0.01) * self.sim_params["period"]
        self.sim_params["dt"] = 2.5e-5 * self.sim_params["period"]
        self.sim_params["total_steps"] = int(self.sim_params["final_time"] / self.sim_params["dt"])
        
        rendering_fps = 60
        self.sim_params["time_step"] = np.float64(float(self.sim_params["final_time"]) / self.sim_params["total_steps"])
        self.sim_params["step_skip"] = int(1.0 / (rendering_fps * self.sim_params["time_step"]))
        self.sim_params["max_rate_of_change_of_activation"] = np.infty
        self.sim_params["no_of_segments"] = 6
        
        self.flagella_sim = FlagellaSimulator()
         # setting up test params
        self.sim_params["n_elem"] = 50
        self.sim_params["NU"] = 5.0 # dissipation coefficient
        self.sim_params["E"] = 1e7  # Young's Modulus
        self.sim_params["start"] = np.zeros((3,))
        self.sim_params["direction_of_rod_extension"] = np.array([0.0, 1.0, 0.0])  # rod direction: pointing upwards
        self.sim_params["normal"] = np.array([0.0, 0.0, 1.0])

        self.sim_params["density"] = 1000
        self.sim_params["poisson_ratio"] = 0.5

        self.sim_params["base_length"] = 0.2  # rod base length
        self.sim_params["radius_tip"] = 0.05  # radius of the rod at the tip
        self.sim_params["radius_base"] = 0.05  # radius of the rod at the base

        self.sim_params["radius_along_rod"] = np.linspace(self.sim_params["radius_base"], self.sim_params["radius_tip"], self.sim_params["n_elem"])

        # Pneumatic Segments creation
        self.shearable_rods = self.segment_creation()
        
        for i in self.shearable_rods: self.flagella_sim.append(i)
        
        # Apply boundary conditions to shearble rod1.
        self.flagella_sim.constrain(self.shearable_rods[0]).using(
            OneEndFixedRod, constrained_position_idx=(0,), constrained_director_idx=(0,)
        )
        
        # Connect shearable rods (pneumatic segments) with each other
        for i in range(self.sim_params["no_of_segments"]-1): self.flagella_sim.connect(first_rod=self.shearable_rods[i], second_rod=self.shearable_rods[i+1], 
                                                                                   first_connect_idx=-1, second_connect_idx=0).using(FixedJoint, k=1e5, nu=0, kt=5e3)

        
        self.control_inp_torque = (np.random.rand(self.sim_params["no_of_segments"]))
        self.control_inp_torque_dir = np.array([[0.0, 0.0, 0.0]]*self.sim_params["no_of_segments"])  # Number_of_segments X No_of_directions
        # Apply torques
        for i in range(self.sim_params["no_of_segments"]) : self.flagella_sim.add_forcing_to(self.shearable_rods[i]).using(
                                                            UniformTorques,
                                                            torque = self.control_inp_torque[i],
                                                            direction = self.control_inp_torque_dir[i]
                                                        )
        
        self.sim_params["phase_shift"] = 0.0
        self.sim_params["ramp_up_time_MuscleTorques"]=self.sim_params["period"]
        # self.sim_params["rest_lengths"]= [i.rest_lengths for i in self.shearable_rods]
        self.sim_params["with_spline"]=True
        self.sim_params["ramp_up_time_EndpointForces"] = 0.0
        self.sim_params["ramp_up_time_EndpointForcesSinusoidal"] = 0.0
        
        # my_spline = [np.ones(np.cumsum(i.rest_lengths).shape) for i in self.shearable_rods]
        # time = 1.0
        # angular_frequency = 2.0 * np.pi / self.sim_params["period"]
        # factor = min(1.0, time / self.sim_params["period"])
        self.sim_params["force"] = 0.0
        self.sim_params["direction_UniformForces"] = np.array([0.0, 0.0, 0.0])
        self.sim_params["torque"] = self.control_inp_torque
        self.sim_params["direction_UniformTorques"] = self.control_inp_torque_dir 
        self.sim_params["start_force"] =  np.array([0.0, 0.0, 0.0])
        self.sim_params["end_force"] =  np.array([0.0, 0.0, 0.0])
        self.sim_params["ramp_up_time_EndpointForces"] = 0.0
        self.sim_params["acc_gravity"] =  np.array([0.0, 0.0, 0.0])
        self.sim_params["start_force_mag"] =  0.0
        self.sim_params["end_force_mag"] =  0.0
        self.sim_params["ramp_up_time_EndpointForcesSinusoidal"] = 0.0
        self.sim_params["tangent_direction"] = np.array([0.0, 0.0, 0.0])
        self.sim_params["normal_direction"] = np.array([0.0, 0.0, 0.0])
        
        # self.sim_params["muscle_torque_mag"] = muscle_torque_mag_cal(factor, my_spline, angular_frequency, time, self.sim_params["wave_length"], self.sim_params["rest_lengths"])
        # self.sim_params["uniformforces_mag"] = uniformforces_mag_cal(self.sim_params["force"],self.sim_params["direction_UniformForces"], self.sim_params["n_elem"])
        # self.sim_params["uniformtorques_mag"] = uniformtorques_mag_cal(self.sim_params["torque"], self.sim_params["direction_UniformTorques"], self.sim_params["n_elem"], self.sim_params["no_of_segments"])
        
        # Add slender body forces
        fluid_density = 1.0
        reynolds_number = 1e-4
        self.sim_params["dynamic_viscosity"] = (
            fluid_density * self.sim_params["base_length"] * self.sim_params["base_length"] / (self.sim_params["period"] * reynolds_number)
        )
        for i in range(self.sim_params["no_of_segments"]) : self.flagella_sim.add_forcing_to(self.shearable_rods[i]).using(
                                                                SlenderBodyTheory, dynamic_viscosity=self.sim_params["dynamic_viscosity"]
                                                            )
        
        self.time_tracker = mp.Value('d', 0.0)
        
        # Add call backs
        class ContinuumFlagellaCallBack(CallBackBaseClass):
            """
            Call back function for continuum snake
            """

            def __init__(self, step_skip: int, callback_params: dict):
                CallBackBaseClass.__init__(self)
                self.every = step_skip
                self.callback_params = callback_params
                self.pp_list_copy = [defaultdict(list)]*no_of_segments


            def make_callback(self, system, time, current_step: int):
            
                if current_step % self.every == 0:
                    qw,qx,qy,qz = [], [], [], []
                    for i in range(n_elements):
                        Q = system.director_collection[..., i]
                        qw.append(np.sqrt(1 + Q[0, 0] + Q[1, 1] + Q[2, 2]) / 2)
                        qx.append((Q[2, 1] - Q[1, 2]) / (4 * qw[i]))
                        qy.append((Q[0, 2] - Q[2, 0]) / (4 * qw[i]))
                        qz.append((Q[1, 0] - Q[0, 1]) / (4 * qw[i]))
                    rod_state[self.callback_params]["orientation_ww"][:] = qw
                    rod_state[self.callback_params]["orientation_xx"][:] = qx
                    rod_state[self.callback_params]["orientation_yy"][:] = qy
                    rod_state[self.callback_params]["orientation_zz"][:] = qz
                    rod_state[self.callback_params]["position_x"][:] = system.position_collection[0]
                    rod_state[self.callback_params]["position_y"][:] = system.position_collection[1]
                    rod_state[self.callback_params]["position_z"][:] = system.position_collection[2]
                    rod_state[self.callback_params]["velocity_x"][:] = system.velocity_collection[0]
                    rod_state[self.callback_params]["velocity_y"][:] = system.velocity_collection[1]
                    rod_state[self.callback_params]["velocity_z"][:] = system.velocity_collection[2]
                    
                    
                    
                    if time >= 10.0:
                        pp_list_file = open("continuum_flagella_"+str((self.callback_params+1))+".dat", "wb")
                        pickle.dump(self.pp_list_copy[self.callback_params], pp_list_file)
                        pp_list_file.close()

                    self.pp_list_copy[self.callback_params]["time"].append(time)
                    self.pp_list_copy[self.callback_params]["step"].append(current_step)
                    self.pp_list_copy[self.callback_params]["position"].append(
                        system.position_collection.copy()
                    )
                    self.pp_list_copy[self.callback_params]["velocity"].append(
                        system.velocity_collection.copy()
                    )
                    self.pp_list_copy[self.callback_params]["avg_velocity"].append(
                        system.compute_velocity_center_of_mass()
                    )
                    self.pp_list_copy[self.callback_params]["center_of_mass"].append(
                        system.compute_position_center_of_mass()
                    )

                    return
        
                        
        self.pp_list = range(self.sim_params["no_of_segments"])
        
        for i in range(self.sim_params["no_of_segments"]) : self.flagella_sim.collect_diagnostics(self.shearable_rods[i]).using(
                                                            ContinuumFlagellaCallBack, step_skip=200, callback_params=self.pp_list[i]
                                                        )
        self.flagella_sim.finalize()
        # do_step, stages_and_updates will be used in step function
        self.do_step, self.stages_and_updates = extend_stepper_interface(
                                self.StatefulStepper,self.flagella_sim
                            )
    
    
    def segment_creation(self):
        shearable_rods =  np.array([None]*self.sim_params["no_of_segments"])
        
        for i in range(self.sim_params["no_of_segments"]):
            shearable_rods[i] = CosseratRod.straight_rod(self.sim_params["n_elem"], self.sim_params["start"],self.sim_params["direction_of_rod_extension"], 
                                                        self.sim_params["normal"], self.sim_params["base_length"],self.sim_params["radius_along_rod"],
                                                        self.sim_params["density"], self.sim_params["NU"],self.sim_params["E"],self.sim_params["poisson_ratio"],
                                                        )
            self.sim_params["start"] = self.sim_params["start"]+ self.sim_params["direction_of_rod_extension"] * self.sim_params["base_length"]
        return shearable_rods
        
        
        
    def time_stepping(self, control_input):
        
        self.control_inp_torque = np.array(control_input["control_torque"][:])
        self.control_inp_torque_dir = np.array(control_input["control_torque_dir"][:]).reshape((no_of_segments,3))
        
        
        self.time_tracker.value = self.do_step(
            self.StatefulStepper,
            self.stages_and_updates,
            self.flagella_sim,
            self.time_tracker.value,
            self.sim_params["time_step"],
            )
        done = False
        if self.time_tracker.value> 10.01:
            done =True
        
        # Checking for NaN positon of the rod. If so, stop the simulation
        invalid_values_condition = []
        # Checking for NaN positon of the rod. If so, stop the simulation
        for i in range(self.sim_params["no_of_segments"]) : invalid_values_condition.append(_isnan_check(self.shearable_rods[i].position_collection))

        if any(invalid_values_condition) == True:
            print(" NaN detected, will exit the simulation")
            for i in range(self.sim_params["no_of_segments"]) :
                self.shearable_rods[i].position_collection = np.zeros(
                    self.shearable_rods[i].position_collection.shape
                )
            done = True
        return done


control_input["control_torque"][:] = (np.random.rand(no_of_segments))
control_input["control_torque_dir"][:] = np.array([[0.0, 0.0, 0.0]]*no_of_segments).ravel()
            
def run_flagella(
         b_coeff,PLOT_FIGURE=False, SAVE_FIGURE=False, SAVE_VIDEO=False, SAVE_RESULTS=False
    ):

    flagella_run = DefineFlagella(b_coeff)
    
    
    print("Total steps", flagella_run.sim_params["total_steps"])
    
    def flagella_stepping():
        done = False
        
        for i in tqdm(range(flagella_run.sim_params["total_steps"])):
            done = flagella_run.time_stepping(control_input)
            if done:
                break
        
    def ros_node():
        rclpy.init(args=None)
        
        elastica_pub_sub = ElasticaPublisherSubscriber(flagella_run.sim_params, rod_state,flagella_run.time_tracker,control_input)
        rclpy.spin(elastica_pub_sub)
        

    #ROS2 Node
    p1 = mp.Process(target=ros_node)
    #Starting the simulation
    p2 = mp.Process(target=flagella_stepping)
  
    # starting process 1
    p1.start()
    # starting process 2
    p2.start()
    
    # wait until process 2 is finished
    p2.join()
    if not p2.is_alive():
        p1.terminate()
    
    pp_list = []
    for i in range(no_of_segments):
        if os.path.exists("continuum_flagella_"+str((i+1))+".dat"):
            pp_list_file = open("continuum_flagella_"+str((i+1))+".dat", "rb")
            pp_list.append(pickle.load(pp_list_file))
    
    if PLOT_FIGURE:
        for i in range(no_of_segments):
            filename_plot = "continuum_flagella_velocity_"+str((i+1))+".png"
            plot_velocity(pp_list[i], flagella_run.sim_params["period"], filename_plot, SAVE_FIGURE)

        if SAVE_VIDEO:
            filename_video = "continuum_flagella_"+str((i+1))+".mp4"
            plot_video(pp_list[i], video_name=filename_video, margin=0.2, fps=200)

    if SAVE_RESULTS:
        pass
    
    else:
        for i in range(no_of_segments):
            if os.path.exists("continuum_flagella_"+str((i+1))+".dat"):
                os.remove("continuum_flagella_"+str((i+1))+".dat")
            else:
                print("The file does not exist")

        # Compute the average forward velocity. These will be used for optimization.
    # [_, _, avg_forward, avg_lateral] = compute_projected_velocity(pp_list, flagella_run.sim_params["period"])

    return  pp_list

    
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
            [avg_forward, _, _] = run_flagella(spline_coefficient,
                PLOT_FIGURE=False,
                SAVE_FIGURE=False,
                SAVE_VIDEO=False,
                SAVE_RESULTS=False,
            )
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
            wave_length = (
                0.3866575573648976 * 1.0
            )  # 1.0 is base length, wave number is 16.25
            t_coeff_optimized = np.hstack((t_coeff_optimized, wave_length))
        else:
            t_coeff_optimized = np.array([17.4, 48.5, 5.4, 14.7, 0.38])

        # run the simulation
        [pp_list] = run_flagella(
            t_coeff_optimized,PLOT_FIGURE, SAVE_FIGURE, SAVE_VIDEO, SAVE_RESULTS
        )

        # print("average forward velocity:", avg_forward)
        # print("average forward lateral:", avg_lateral)

if __name__ == "__main__":
    
    main()
