__doc__ = """
For  deriving a standard Piecewise Constant Curvature (PCC) kinematic description of the continuum robot's state
"""
import numpy as np
import rclpy
from rclpy.node import Node
from elastica_msgs.msg import *
from std_msgs.msg import Float64MultiArray
import math as m

##print("HEYYYYYY", np.sum(system.lengths), np.sum(system.rest_lengths)), #l(length)
#θ=cos−1(2*⟨q1,q2⟩^2−1)  ⟨q1,q2⟩ is the innner product (a1a2+b1b2+c1c2+d1d2) (phi eq)
#math.degrees(math.acos((np.dot(np.array(system.director_collection[1,:,0]),np.array(system.director_collection[1,:,-1])))/(np.linalg.norm(np.array(system.director_collection[1,:,0])*np.linalg.norm(np.array(system.director_collection[1,:,-1]))))))  #THETA(kappa)

##########ROS2######################################################
class PccKinematics(Node):
    
    def __init__(self):
        super().__init__('rods_state_to_pcc')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('queue_size', None),
                ('print_params', None),
                ('pub_frequency', None),
                ('topic_names', ['elastica/control_input','elastica/time_tracker','elastica/rods_state','elastica/objs_state','elastica/pcc_kinematic_states'])
            ])
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.print_params = self.get_parameter('print_params').get_parameter_value().integer_value
        self.pub_frequency = self.get_parameter('pub_frequency').get_parameter_value().double_value
        self.topic_names = self.get_parameter('topic_names').get_parameter_value().string_array_value
        
        self.no_of_segments = 6 #Number of Pneumatic chambers in the Continuum robot arm
        self.no_of_objects = 1 #Number of objects simualted in ELastica
        self.count = 0
        
        self.phi_arr, self.theta_arr, self.L_arr = [],[],[]

        self.pcc_kin_state_msg = PccKinematicStates()

        self.publisher_pcc_kin_state  =  self.create_publisher(PccKinematicStates, self.topic_names[4], self.queue_size)
        
        
        # self.control_input, self.control_torque_dir =  self.sampleControlTorque()
        timer_period = 1/self.pub_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
        self.subscription_sim_time = self.create_subscription(SimulationTime,self.topic_names[1],self.listener_callback_control_input_change,self.queue_size)
        self.subscription_rod_state = self.create_subscription(RodsState, self.topic_names[2],self.listener_callback_rods_state,self.queue_size)
        
        # prevent unused variable warning
        self.subscription_sim_time
        self.subscription_rod_state
        
        
    def transformation(self,phi_arr, theta_arr, L_arr ):
        """
        n homogenous transformations for n segmnets. 
        Describes the movement of the pneumatically actuated robot segment 
        if it is bending with a constant curvature.
        maps each reference system to subsequent one and describes robot's kinematics
        transformations include both rotation and translation
        
        input args: 
                phi_arr: array of phi (radians) for all segments, theta_arr: array of theta (radians) for all segments, L_arr: array of lengths of curvatures for all segments
        Returns
        array of size (n_segments,4,4), 'n_segments' transformation matrices (4 X 4) each for the joint cosserat rods

        -------
        """
        transformations = np.empty((self.no_of_segments,4,4))

        for i in range(self.no_of_segments):
            transformations[i] = np.array([[(m.cos(phi_arr[i]))**2*(m.cos(theta_arr[i])-1)+1, (m.sin(phi_arr[i])*m.cos(phi_arr[i])*(m.cos(theta_arr[i])-1)), (m.cos(phi_arr[i])*m.sin(theta_arr[i])), ((m.cos(phi_arr[i])*(1-m.cos(theta_arr[i])))*(L_arr[i]/theta_arr[i]))],
                                       [(m.sin(phi_arr[i])*m.cos(phi_arr[i])*(m.cos(theta_arr[i])-1)), ((m.sin(phi_arr[i]))**2*(m.cos(theta_arr[i])-1)+1), (m.sin(phi_arr[i])*m.sin(theta_arr[i])), ((m.sin(phi_arr[i])*(1-m.cos(theta_arr[i])))*(L_arr[i]/theta_arr[i]))],
                                       [(-m.cos(phi_arr[i])*m.sin(theta_arr[i])), (-m.sin(phi_arr[i])*m.sin(theta_arr[i])), (m.cos(theta_arr[i])), ((m.sin(theta_arr[i]))*(L_arr[i]/theta_arr[i]))],
                                       [0,0,0,1]])

        
        return transformations

    def timer_callback(self):
        transformations = self.transformation(self.phi_arr, self.theta_arr, self.L_arr)
        for i in range(self.no_of_segments):
            transformation = Float64MultiArray()
            transformation.data = transformations[i].ravel().tolist()  #Have to reshaped into (4 X 4) when used for transformation
            self.pcc_kin_state_msg.transformations.append(transformation) 
        self.publisher_pcc_kin_state.publish(self.pcc_kin_state_msg)
        
    def listener_callback_rods_state(self, msg):
  
        for i in range(self.no_of_segments):
            
            #phi calculation
            quaternion_base = np.array([msg.rod_states[i].poses[0].pose.orientation.w, msg.rod_states[i].poses[0].pose.orientation.x, \
                                msg.rod_states[i].poses[0].pose.orientation.y, msg.rod_states[i].poses[0].pose.orientation.z])
            quaternion_tip = np.array([msg.rod_states[i].poses[-1].pose.orientation.w, msg.rod_states[i].poses[-1].pose.orientation.x, \
                                msg.rod_states[i].poses[-1].pose.orientation.y, msg.rod_states[i].poses[-1].pose.orientation.z])
            phi_i = m.acos((2*((np.dot(quaternion_base,quaternion_tip))**2))-1) 
            self.phi_arr.append(phi_i)
            
            #theta calculation
            theta_i = m.acos((np.dot(np.array(msg.rod_states[i].normal_director_base.data),np.array(msg.rod_states[i].normal_director_tip.data)))/ \
                      (np.linalg.norm(np.array(msg.rod_states[i].normal_director_base.data)*np.linalg.norm(np.array(msg.rod_states[i].normal_director_tip.data)))))
            self.theta_arr.append(theta_i)
            
            #length of curve calculation
            
            L_i = np.sum(np.array(msg.rod_states[i].lengths.data))
            self.L_arr.append(L_i)
    
 
                
        
    def listener_callback_control_input_change(self, msg):
        
        pass

def main():
    rclpy.init(args=None)
    pcc_kinematics = PccKinematics()
    rclpy.spin(pcc_kinematics)
    
if __name__== "__main__":
    main()
#############################################################################
