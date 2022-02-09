__doc__ = """
For  deriving a standard Piecewise Constant Curvature (PCC) kinematic description of the continuum robot's state
"""
import numpy as np
import rclpy
from rclpy.node import Node
from elastica_msgs.msg import *




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

        self.pcc_kin_state_msg = PccKinematicStates()

        self.publisher_pcc_kin_state  =  self.create_publisher(PccKinematicStates, self.topic_names[4], self.queue_size)
        
        
        # self.control_input, self.control_torque_dir =  self.sampleControlTorque()
        timer_period = 1/self.pub_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
        self.subscription_sim_time = self.create_subscription(SimulationTime,self.topic_names[1],self.listener_callback_control_input_change,self.queue_size)
        self.subscription_rod_state = self.create_subscription(RodsState, self.topic_names[2],self.listener_callback_rods_state,self.queue_size)
        self.subscription_objs_state = self.create_subscription(ObjsState, self.topic_names[3],self.listener_callback_objs_state,self.queue_size)
        
        # prevent unused variable warning
        self.subscription_sim_time
        self.subscription_rod_state
        self.subscription_objs_state
        
        
    def transformation(self,phi_i, theta_i, L_i ):
        """
        n homogenous transformations for n segmnets. 
        Describes the movement of the pneumatically actuated robot segment 
        if it is bending with a constant curvature.
        maps each reference system to subsequent one and describes robot's kinematics
        
        Returns
        -------
        """
        
        random_control_torque = (np.random.rand(self.no_of_segments))
        
        random_torque_dir = []
        for i in range(self.no_of_segments):
            num_dir_torque_on = np.random.randint(0,4)
            num_dir_torque_off = 3-num_dir_torque_on
            random_torque_dir.append(np.array([1.0]*num_dir_torque_on + [0.0]*num_dir_torque_off))
            np.random.shuffle(random_torque_dir[i])
        
                
        return random_control_torque, np.array(random_torque_dir)

    def timer_callback(self):
        
        # self.pcc_kin_state_msg.control_torques.data =  np.squeeze(self.control_input).tolist()
        # self.control_input_msg.control_torque_dir.data =  (self.control_torque_dir).ravel().tolist()
        
        self.publisher_pcc_kin_state.publish(self.pcc_kin_state_msg)
        
    def listener_callback_rods_state(self, msg):
        # if self.print_params: 
        #     for i in range(msg.num_rods):
        #         self.get_logger().info("I heard seg'"+str(i+1)+"s elements pose: "+ (str(msg.rod_states[i].poses)))
        #         self.get_logger().info("I heard seg'"+str(i+1)+"s elements velocity: "+ (str(msg.rod_states[i].velocities)))
        #         self.get_logger().info("I heard seg'"+str(i+1)+"s kappa: "+ (str(msg.rod_states[i].kappa)))
        #         self.get_logger().info(5*"\n")
        pass
    
    def listener_callback_objs_state(self, msg):
        # obj_names = msg.obj_names.split(',')
        # if self.print_params: 
        #     for i in range(msg.num_objects):
        #         self.get_logger().info("I heard "+obj_names[i]+"'s pose: "+ (str(msg.obj_poses[i])))
        #         self.get_logger().info(5*"\n")
        pass
                
        
    def listener_callback_control_input_change(self, msg):
        # if self.print_params: 
        #     self.get_logger().info("I heard control points"+ (str(msg.current_sim_time)))
        
        # if int(msg.current_sim_time) % 3 == 0 and self.count ==0 and (msg.current_sim_time)>3.0 :
        #         self.control_input, self.control_torque_dir  = self.sampleControlTorque()
        #         self.count = 1
        #         self.get_logger().info("CHANGING CONTROL TORQUE")
        # if self.count==1 and msg.current_sim_time % 3>2.9:
        #         self.count =0
        pass

def main():
    rclpy.init(args=None)
    pcc_kinematics = PccKinematics()
    rclpy.spin(pcc_kinematics)
    
if __name__== "__main__":
    main()
#############################################################################