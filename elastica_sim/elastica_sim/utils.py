__doc__ = """
Includes utility functions for calculation of applied force & torque magnitudes on the cosserat rods simulated
"""
import numpy as np

    
def muscle_torque_mag_cal(factor,my_spline,angular_frequency,time,wave_length,rest_lengths):
    muscle_torque_mag = (
    factor
    * my_spline
    * np.sin(angular_frequency * time - (2.0 * np.pi / (wave_length)) * np.cumsum(rest_lengths) + 0.0)
    ) #Vector of dim equal to n_elem(number of elements in the cosserat rod)
    return muscle_torque_mag
    
def uniformforces_mag_cal(force,direction_UniformForces,n_elem):
    uniformforces_mag = ((force * direction_UniformForces).reshape(3, 1))/n_elem  #Uniform force on one element
    return uniformforces_mag
        
def uniformtorques_mag_cal(torque,direction_UniformTorques,n_elem):
    uniformtorques_mag =   (torque * direction_UniformTorques) / n_elem #Uniform torque on one element
    return uniformtorques_mag