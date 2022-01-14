__doc__ = """Continuum snake example, for detailed explanation refer to Gazzola et. al. R. Soc. 2018  
section 5.2 
HERE MUSCLE TORQUE WITH CHANGING BETA SPLINE HAS BEEN IMPLEMENTED
"""

import numpy as np
import sys
from elastica.timestepper import extend_stepper_interface
from elastica._calculus import _isnan_check
from tqdm import tqdm


# FIXME without appending sys.path make it more generic
sys.path.append("../../")
import os
from elastica import *
from examples.ContinuumSnakeCase.continuum_snake_postprocessing import (
    plot_snake_velocity,
    plot_video,
    compute_projected_velocity,
)

from examples.ContinuumSnakeCase import MuscleTorquesWithVaryingBetaSplines


class SnakeSimulator(BaseSystemCollection, Constraints, Forcing, CallBacks):
    pass

class DefineSnake():
    def __init__(self, b_coeff):
        self.StatefulStepper = PositionVerlet()
        self.period = 1.0
        self.b_coeff = b_coeff[-1]
        self.wave_length = b_coeff[-1]
        self.number_of_control_points = 6
        self.alpha = 75
        
        self.final_time = (10.0 + 0.01) * self.period
        self.dt = 2.5e-5 * self.period
        self.total_steps = int(self.final_time / self.dt)
        
        rendering_fps = 60
        self.time_step = np.float64(float(self.final_time) / self.total_steps)
        self.step_skip = int(1.0 / (rendering_fps * self.time_step))
        self.max_rate_of_change_of_activation = np.infty
        
        self.snake_sim = SnakeSimulator()

        # setting up test params
        self.n_elem = 50
        self.start = np.zeros((3,))
        self.direction = np.array([0.0, 0.0, 1.0])
        self.normal = np.array([0.0, 1.0, 0.0])
        self.base_length = 1.0
        self.base_radius = 0.025
        self.density = 1000
        self.nu = 5.0
        self.E = 1e7
        self.poisson_ratio = 0.5

        self.shearable_rod = CosseratRod.straight_rod(
            self.n_elem,
            self.start,
            self.direction,
            self.normal,
            self.base_length,
            self.base_radius,
            self.density,
            self.nu,
            self.E,
            self.poisson_ratio,
        )

        self.snake_sim.append(self.shearable_rod)

        # Add gravitational forces
        self.gravitational_acc = -9.80665
        self.snake_sim.add_forcing_to(self.shearable_rod).using(
            GravityForces, acc_gravity=np.array([0.0, self.gravitational_acc, 0.0])
        )

        self.torque_profile_list_for_muscle_in_normal_dir = defaultdict(list)
        self.control_points_array_normal_dir = []
        # Apply torques
        self.snake_sim.add_forcing_to(self.shearable_rod).using(
            MuscleTorquesWithVaryingBetaSplines,
            base_length=self.base_length,
            number_of_control_points=self.number_of_control_points,
            points_func_array=self.control_points_array_normal_dir,
            muscle_torque_scale=self.alpha,
            direction=str("normal"),
            step_skip=self.step_skip,
            max_rate_of_change_of_activation=self.max_rate_of_change_of_activation,
            torque_profile_recorder=self.torque_profile_list_for_muscle_in_normal_dir,
        )
        # Add friction forces
        self.origin_plane = np.array([0.0, -self.base_radius, 0.0])
        self.normal_plane = self.normal
        self.slip_velocity_tol = 1e-8
        self.froude = 0.1
        self.mu = self.base_length / (self.period * self.period * np.abs(self.gravitational_acc) * self.froude)
        kinetic_mu_array = np.array(
            [self.mu, 1.5 * self.mu, 2.0 * self.mu]
        )  # [forward, backward, sideways]
        static_mu_array = 2 * kinetic_mu_array
        self.snake_sim.add_forcing_to(self.shearable_rod).using(
            AnisotropicFrictionalPlane,
            k=1.0,
            nu=1e-6,
            plane_origin=self.origin_plane,
            plane_normal=self.normal_plane,
            slip_velocity_tol=self.slip_velocity_tol,
            static_mu_array=static_mu_array,
            kinetic_mu_array=kinetic_mu_array,
        )

        # Add call backs
        class ContinuumSnakeCallBack(CallBackBaseClass):
            """
            Call back function for continuum snake
            """

            def __init__(self, step_skip: int, callback_params: dict):
                CallBackBaseClass.__init__(self)
                self.every = step_skip
                self.callback_params = callback_params

            def make_callback(self, system, time, current_step: int):

                if current_step % self.every == 0:

                    self.callback_params["time"].append(time)
                    self.callback_params["step"].append(current_step)
                    self.callback_params["position"].append(
                        system.position_collection.copy()
                    )
                    self.callback_params["velocity"].append(
                        system.velocity_collection.copy()
                    )
                    self.callback_params["avg_velocity"].append(
                        system.compute_velocity_center_of_mass()
                    )

                    self.callback_params["center_of_mass"].append(
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

        self.time_tracker = np.float64(0.0)
    
    def time_stepping(self, action):
        
        self.action = action
        self.control_points_array_normal_dir[:] = action[
                : self.number_of_control_points
            ]
        
        
        self.time_tracker = self.do_step(
            self.StatefulStepper,
            self.stages_and_updates,
            self.snake_sim,
            self.time_tracker,
            self.time_step,
            )
        done = False
        if self.time_tracker> 10.01:
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
    
def sampleControlPoints(number_of_control_points):
    """
    Sample usable random control point values.
    Returns
    -------
    numpy.ndarray
        1D (3 * number_of_control_points,) array containing data with 'float' type, in range [-1, 1].
    """
    random_control_points = []
    random_control_points[:] = ((np.random.rand(3 * number_of_control_points) - 0.5) * 2)
            
    return random_control_points

def run_snake(
    b_coeff, PLOT_FIGURE=False, SAVE_FIGURE=False, SAVE_VIDEO=False, SAVE_RESULTS=False
    ):
    
    snake_run = DefineSnake(b_coeff)
    count = 0
    done = False
    action  = sampleControlPoints(6)[:6]
    
    print("Total steps", snake_run.total_steps)
    
    for i in tqdm(range(snake_run.total_steps)):
        if int(snake_run.time_tracker) % 3 == 0 and count ==0 and (snake_run.time_tracker)>3.0 :
            action = sampleControlPoints(6)[:6]
            count = 1
            print ("CHANGING CONTROL")
        if count==1 and snake_run.time_tracker % 3>2.9:
            count =0
        done = snake_run.time_stepping(action)
        if done:
            break

    if PLOT_FIGURE:
        filename_plot = "continuum_snake_velocity.png"
        plot_snake_velocity(snake_run.pp_list, snake_run.period, filename_plot, SAVE_FIGURE)

        if SAVE_VIDEO:
            filename_video = "continuum_snake.mp4"
            plot_video(snake_run.pp_list, video_name=filename_video, margin=0.2, fps=500)

    if SAVE_RESULTS:
        import pickle

        filename = "continuum_snake.dat"
        file = open(filename, "wb")
        pickle.dump(snake_run.pp_list, file)
        file.close()

    # Compute the average forward velocity. These will be used for optimization.
    [_, _, avg_forward, avg_lateral] = compute_projected_velocity(snake_run.pp_list, snake_run.period)

    return avg_forward, avg_lateral, snake_run.pp_list


if __name__ == "__main__":

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

################################################################MUSCLETORQUE AS A TRAVELLING WAVE################################################################################################
# __doc__ = """Continuum snake example, for detailed explanation refer to Gazzola et. al. R. Soc. 2018  
# section 5.2 """

# import numpy as np
# import sys

# # FIXME without appending sys.path make it more generic
# sys.path.append("../../")
# import os
# from elastica import *
# from examples.ContinuumSnakeCase.continuum_snake_postprocessing import (
#     plot_snake_velocity,
#     plot_video,
#     compute_projected_velocity,
# )


# class SnakeSimulator(BaseSystemCollection, Constraints, Forcing, CallBacks):
#     pass


# def run_snake(
#     b_coeff, PLOT_FIGURE=False, SAVE_FIGURE=False, SAVE_VIDEO=False, SAVE_RESULTS=False
# ):

#     snake_sim = SnakeSimulator()

#     # setting up test params
#     n_elem = 50
#     start = np.zeros((3,))
#     direction = np.array([0.0, 0.0, 1.0])
#     normal = np.array([0.0, 1.0, 0.0])
#     base_length = 1.0
#     base_radius = 0.025
#     density = 1000
#     nu = 5.0
#     E = 1e7
#     poisson_ratio = 0.5

#     shearable_rod = CosseratRod.straight_rod(
#         n_elem,
#         start,
#         direction,
#         normal,
#         base_length,
#         base_radius,
#         density,
#         nu,
#         E,
#         poisson_ratio,
#     )

#     snake_sim.append(shearable_rod)

#     # Add gravitational forces
#     gravitational_acc = -9.80665
#     snake_sim.add_forcing_to(shearable_rod).using(
#         GravityForces, acc_gravity=np.array([0.0, gravitational_acc, 0.0])
#     )

#     period = 1.0
#     wave_length = b_coeff[-1]
#     snake_sim.add_forcing_to(shearable_rod).using(
#         MuscleTorques,
#         base_length=base_length,
#         b_coeff=b_coeff[:-1],
#         period=period,
#         wave_number=2.0 * np.pi / (wave_length),
#         phase_shift=0.0,
#         rest_lengths=shearable_rod.rest_lengths,
#         ramp_up_time=period,
#         direction=normal,
#         with_spline=True,
#     )

#     # Add friction forces
#     origin_plane = np.array([0.0, -base_radius, 0.0])
#     normal_plane = normal
#     slip_velocity_tol = 1e-8
#     froude = 0.1
#     mu = base_length / (period * period * np.abs(gravitational_acc) * froude)
#     kinetic_mu_array = np.array(
#         [mu, 1.5 * mu, 2.0 * mu]
#     )  # [forward, backward, sideways]
#     static_mu_array = 2 * kinetic_mu_array
#     snake_sim.add_forcing_to(shearable_rod).using(
#         AnisotropicFrictionalPlane,
#         k=1.0,
#         nu=1e-6,
#         plane_origin=origin_plane,
#         plane_normal=normal_plane,
#         slip_velocity_tol=slip_velocity_tol,
#         static_mu_array=static_mu_array,
#         kinetic_mu_array=kinetic_mu_array,
#     )

#     # Add call backs
#     class ContinuumSnakeCallBack(CallBackBaseClass):
#         """
#         Call back function for continuum snake
#         """

#         def __init__(self, step_skip: int, callback_params: dict):
#             CallBackBaseClass.__init__(self)
#             self.every = step_skip
#             self.callback_params = callback_params

#         def make_callback(self, system, time, current_step: int):

#             if current_step % self.every == 0:

#                 self.callback_params["time"].append(time)
#                 self.callback_params["step"].append(current_step)
#                 self.callback_params["position"].append(
#                     system.position_collection.copy()
#                 )
#                 self.callback_params["velocity"].append(
#                     system.velocity_collection.copy()
#                 )
#                 self.callback_params["avg_velocity"].append(
#                     system.compute_velocity_center_of_mass()
#                 )

#                 self.callback_params["center_of_mass"].append(
#                     system.compute_position_center_of_mass()
#                 )

#                 return

#     pp_list = defaultdict(list)
#     snake_sim.collect_diagnostics(shearable_rod).using(
#         ContinuumSnakeCallBack, step_skip=200, callback_params=pp_list
#     )

#     snake_sim.finalize()
#     timestepper = PositionVerlet()
#     # timestepper = PEFRL()

#     final_time = (11.0 + 0.01) * period
#     dt = 1.0e-3 * period
#     total_steps = int(final_time / dt)
#     print("Total steps", total_steps)
#     integrate(timestepper, snake_sim, final_time, total_steps)

#     if PLOT_FIGURE:
#         filename_plot = "continuum_snake_velocity.png"
#         plot_snake_velocity(pp_list, period, filename_plot, SAVE_FIGURE)

#         if SAVE_VIDEO:
#             filename_video = "continuum_snake.mp4"
#             plot_video(pp_list, video_name=filename_video, margin=0.2, fps=500)

#     if SAVE_RESULTS:
#         import pickle

#         filename = "continuum_snake.dat"
#         file = open(filename, "wb")
#         pickle.dump(pp_list, file)
#         file.close()

#     # Compute the average forward velocity. These will be used for optimization.
#     [_, _, avg_forward, avg_lateral] = compute_projected_velocity(pp_list, period)

#     return avg_forward, avg_lateral, pp_list


# if __name__ == "__main__":

#     # Options
#     PLOT_FIGURE = True
#     SAVE_FIGURE = False
#     SAVE_VIDEO = False
#     SAVE_RESULTS = False
#     CMA_OPTION = False

#     if CMA_OPTION:
#         import cma

#         SAVE_OPTIMIZED_COEFFICIENTS = False

#         def optimize_snake(spline_coefficient):
#             [avg_forward, _, _] = run_snake(
#                 spline_coefficient,
#                 PLOT_FIGURE=False,
#                 SAVE_FIGURE=False,
#                 SAVE_VIDEO=False,
#                 SAVE_RESULTS=False,
#             )
#             return -avg_forward

#         # Optimize snake for forward velocity. In cma.fmin first input is function
#         # to be optimized, second input is initial guess for coefficients you are optimizing
#         # for and third input is standard deviation you initially set.
#         optimized_spline_coefficients = cma.fmin(optimize_snake, 5 * [0], 0.5)

#         # Save the optimized coefficients to a file
#         filename_data = "optimized_coefficients.txt"
#         if SAVE_OPTIMIZED_COEFFICIENTS:
#             assert filename_data != "", "provide a file name for coefficients"
#             np.savetxt(filename_data, optimized_spline_coefficients, delimiter=",")

#     else:
#         # Add muscle forces on the rod
#         if os.path.exists("optimized_coefficients.txt"):
#             t_coeff_optimized = np.genfromtxt(
#                 "optimized_coefficients.txt", delimiter=","
#             )
#             wave_length = 0.97 * 1.0  # 1.0 is base length
#             t_coeff_optimized = np.hstack((t_coeff_optimized, wave_length))

#         else:
#             t_coeff_optimized = np.array([17.4, 48.5, 5.4, 14.7, 0.97])

#         # run the simulation
#         [avg_forward, avg_lateral, pp_list] = run_snake(
#             t_coeff_optimized, PLOT_FIGURE, SAVE_FIGURE, SAVE_VIDEO, SAVE_RESULTS
#         )

#         print("average forward velocity:", avg_forward)
#         print("average forward lateral:", avg_lateral)