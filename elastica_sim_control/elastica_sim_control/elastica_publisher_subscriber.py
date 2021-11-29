__doc__ = """
For creating publishers & subscribers of ros topics for various simulation parameters, applied force & torque magnitudes 
& state space of cosserat rods simulated
"""
import numpy as np
from rclpy.node import Node
from elastica_msgs.msg import *


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

##########ROS2######################################################
class ElasticaPublisherSubscriber(Node):
    
    def __init__(self, sim_params, rod_state, print_params, time_tracker, control_input):
        super().__init__('elastica_pub_sub')
        self.print_params = print_params
        self.sim_params = sim_params
        self.rod_state = rod_state
        self.count = 0

        self.physical_params = PhysicalParams()
        self.rod_state_msg = RodState()
        self.control_input_msg = ControlInput()

        self.publisher0 = self.create_publisher(PhysicalParams, '/physical_params', 10)
        self.publisher1 = self.create_publisher(RodState, '/rod_state', 10)
        self.publisher2  =  self.create_publisher(ControlInput, '/control_input', 10)
        self.control_input = control_input
        self.time_tracker = time_tracker
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription0 = self.create_subscription(PhysicalParams,'/physical_params',self.listener_callback_physical_params,10)
        self.subscription1 = self.create_subscription(RodState,'/rod_state',self.listener_callback_rod_state,10)
        self.subscription2 = self.create_subscription(ControlInput,'/control_input',self.listener_callback_control_input,10)
        
        # prevent unused variable warning
        self.subscription0
        self.subscription1
        self.subscription2
        


    def timer_callback(self):
        
 
        self.physical_params.b_coeff.data  = self.sim_params["b_coeff"].tolist()
        self.physical_params.period = self.sim_params["period"]
        self.physical_params.wave_length = self.sim_params["wave_length"]
        self.physical_params.base_length = self.sim_params["base_length"]
        self.physical_params.phase_shift = self.sim_params["phase_shift"]
        self.physical_params.rest_lengths.data = self.sim_params["rest_lengths"].tolist()
        self.physical_params.ramp_up_time_muscle_torques = self.sim_params["ramp_up_time_MuscleTorques"]
        self.physical_params.direction_of_rod_extension.x = self.sim_params["direction_of_rod_extension"].tolist()[0]
        self.physical_params.direction_of_rod_extension.y = self.sim_params["direction_of_rod_extension"].tolist()[1]
        self.physical_params.direction_of_rod_extension.z = self.sim_params["direction_of_rod_extension"].tolist()[2]
        self.physical_params.with_spline = self.sim_params["with_spline"]
        self.physical_params.muscle_torque_mag.data = self.sim_params["muscle_torque_mag"].tolist()
        self.physical_params.force = self.sim_params["force"]
        self.physical_params.direction_uniform_forces.x = self.sim_params["direction_UniformForces"].tolist()[0]
        self.physical_params.direction_uniform_forces.y = self.sim_params["direction_UniformForces"].tolist()[1]
        self.physical_params.direction_uniform_forces.z = self.sim_params["direction_UniformForces"].tolist()[2]
        self.physical_params.uniformforces_mag.data = np.squeeze(self.sim_params["uniformforces_mag"]).tolist()
        self.physical_params.torque = self.sim_params["torque"]
        self.physical_params.direction_uniform_torques.x = self.sim_params["direction_UniformTorques"].tolist()[0]
        self.physical_params.direction_uniform_torques.y = self.sim_params["direction_UniformTorques"].tolist()[1]
        self.physical_params.direction_uniform_torques.z = self.sim_params["direction_UniformTorques"].tolist()[2]
        self.physical_params.uniformtorques_mag.data = np.squeeze(self.sim_params["uniformtorques_mag"]).tolist()
        self.physical_params.start_force.force.x = self.sim_params["start_force"].tolist()[0]
        self.physical_params.start_force.force.y = self.sim_params["start_force"].tolist()[1]
        self.physical_params.start_force.force.z = self.sim_params["start_force"].tolist()[2]
        self.physical_params.end_force.force.x = self.sim_params["end_force"].tolist()[0]
        self.physical_params.end_force.force.y = self.sim_params["end_force"].tolist()[1]
        self.physical_params.end_force.force.z = self.sim_params["end_force"].tolist()[2]
        self.physical_params.ramp_up_time_endpoint_forces = self.sim_params["ramp_up_time_EndpointForces"]
        self.physical_params.acc_gravity.linear.x = self.sim_params["acc_gravity"].tolist()[0]
        self.physical_params.acc_gravity.linear.y = self.sim_params["acc_gravity"].tolist()[1]
        self.physical_params.acc_gravity.linear.z = self.sim_params["acc_gravity"].tolist()[2]
        self.physical_params.dynamic_viscosity = self.sim_params["dynamic_viscosity"]
        self.physical_params.start_force_mag = self.sim_params["start_force_mag"]
        self.physical_params.end_force_mag = self.sim_params["end_force_mag"]
        self.physical_params.ramp_up_time_endpoint_forces_sinusoidal = self.sim_params["ramp_up_time_EndpointForcesSinusoidal"]
        self.physical_params.tangent_direction.x = self.sim_params["tangent_direction"].tolist()[0]
        self.physical_params.tangent_direction.y = self.sim_params["tangent_direction"].tolist()[1]
        self.physical_params.tangent_direction.z = self.sim_params["tangent_direction"].tolist()[2]
        self.physical_params.normal_direction.x = self.sim_params["normal_direction"].tolist()[0]
        self.physical_params.normal_direction.y = self.sim_params["normal_direction"].tolist()[1]
        self.physical_params.normal_direction.z = self.sim_params["normal_direction"].tolist()[2]
        self.physical_params.rod_tip_orientation.w = np.squeeze(self.rod_state["rod_tip_orientation"]).tolist()[0]
        self.physical_params.rod_tip_orientation.x = np.squeeze(self.rod_state["rod_tip_orientation"]).tolist()[1]
        self.physical_params.rod_tip_orientation.y = np.squeeze(self.rod_state["rod_tip_orientation"]).tolist()[2]
        self.physical_params.rod_tip_orientation.z = np.squeeze(self.rod_state["rod_tip_orientation"]).tolist()[3]
        self.rod_state_msg.position_x.data = np.squeeze(self.rod_state["position_x"]).tolist()
        self.rod_state_msg.position_y.data = np.squeeze(self.rod_state["position_y"]).tolist()
        self.rod_state_msg.position_z.data = np.squeeze(self.rod_state["position_z"]).tolist()
        self.rod_state_msg.velocity_x.data = np.squeeze(self.rod_state["velocity_x"]).tolist()
        self.rod_state_msg.velocity_y.data = np.squeeze(self.rod_state["velocity_y"]).tolist()
        self.rod_state_msg.velocity_z.data = np.squeeze(self.rod_state["velocity_z"]).tolist()
        
        self.control_input_msg.control_points.data =  np.squeeze(self.control_input["control_points"]).tolist()
        
        self.publisher0.publish(self.physical_params)
        self.publisher1.publish(self.rod_state_msg)
        self.publisher2.publish(self.control_input_msg)
        

    def listener_callback_physical_params(self, msg):
        if self.print_params: print('I heard rod tip orientation:', [msg.rod_tip_orientation.w,msg.rod_tip_orientation.x,msg.rod_tip_orientation.y,msg.rod_tip_orientation.z])
    def listener_callback_rod_state(self, msg):
        if self.print_params: 
            print("I heard rod_state's position_y:", msg.position_x)
            print("I heard rod_state's position_y:", msg.position_y)
            print("I heard rod_state's position_z:", msg.position_z)
            print("I heard rod_state's velocity_x:", msg.velocity_x)
            print("I heard rod_state's velocity_y:", msg.velocity_y)
            print("I heard rod_state's velocity_z:", msg.velocity_z)
    def listener_callback_control_input(self, msg):
        print("I heard control points", msg.control_points)
        
        if int(self.time_tracker.value) % 3 == 0 and self.count ==0 and (self.time_tracker.value)>3.0 :
                self.control_input["control_points"][:] = sampleControlPoints(6)
                self.count = 1
                print ("CHANGING CONTROL")
        if self.count==1 and self.time_tracker.value % 3>2.9:
                self.count =0
        
    
#############################################################################