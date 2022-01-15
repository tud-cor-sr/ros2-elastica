__doc__ = """
For creating publishers & subscribers of ros topics for various simulation parameters, applied force & torque magnitudes 
& state space of cosserat rods simulated
"""
import numpy as np
import rclpy
from rclpy.node import Node
from elastica_msgs.msg import *




##########ROS2######################################################
class ElasticaControl(Node):
    
    def __init__(self):
        super().__init__('elastica_random_control')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('queue_size', None),
                ('print_params', None),
                ('pub_frequency', None)
            ])
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.print_params = self.get_parameter('print_params').get_parameter_value().integer_value
        self.pub_frequency = self.get_parameter('pub_frequency').get_parameter_value().double_value
        
        self.no_of_segments = 6 #Number of Pneumatic chambers in the Continuum robot arm
        self.count = 0

        self.control_input_msg = ControlInput()

        self.publisher_control_inp  =  self.create_publisher(ControlInput, '/control_input', self.queue_size)
        
        
        self.control_input, self.control_torque_dir =  self.sampleControlTorque()
        timer_period = 1/self.pub_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
        self.subscription_sim_time = self.create_subscription(SimulationTime,'/time_tracker',self.listener_callback_control_input_change,self.queue_size)
        self.subscription_rod_state = self.create_subscription(RodState, '/rod_state',self.listener_callback_rod_state,self.queue_size)
        
        # prevent unused variable warning
        self.subscription_sim_time
        self.subscription_rod_state
        
        
    def sampleControlTorque(self):
        """
        Sample usable random torque values for each direcitonal vector.
        Returns
        -------
        numpy.ndarray
            1D array (no_of_segments,) containing data with 'float' type. Total torque applied to a rod-like object.
        numpy.ndarray
            2D array (number_of_segments, no_of_directions) containing data with 'float' type. Directions in which total torque applied to a rod-like object.
        """
        
        random_control_torque = (np.random.rand(self.no_of_segments))
        
        random_torque_dir = []
        for i in range(self.no_of_segments):
            no_dir_torque_on = np.random.randint(0,4)
            no_dir_torque_off = 3-no_dir_torque_on
            random_torque_dir.append(np.array([1.0]*no_dir_torque_on + [0.0]*no_dir_torque_off))
            np.random.shuffle(random_torque_dir[i])
        
                
        return random_control_torque, np.array(random_torque_dir)

    def timer_callback(self):
        
        self.control_input_msg.control_torques.data =  np.squeeze(self.control_input).tolist()
        self.control_input_msg.control_torque_dir.data =  (self.control_torque_dir).ravel().tolist()
        
        self.publisher_control_inp.publish(self.control_input_msg)
        
    def listener_callback_rod_state(self, msg):
        if self.print_params: 
            self.get_logger().info("I heard seg1's position_x: "+ (str(msg.position_x_1.data)))
            self.get_logger().info("I heard seg1's position_y: "+ (str(msg.position_y_1.data)))
            self.get_logger().info("I heard seg1's position_z: "+  (str(msg.position_z_1.data)))
            self.get_logger().info("I heard seg1's velocity_x: "+  (str(msg.velocity_x_1.data)))
            self.get_logger().info("I heard seg1's velocity_y: "+  (str(msg.velocity_y_1.data)))
            self.get_logger().info("I heard seg1's velocity_z: "+  (str(msg.velocity_z_1.data)))
            
            self.get_logger().info("I heard seg2's position_x: "+ (str(msg.position_x_2.data)))
            self.get_logger().info("I heard seg2's position_y: "+ (str(msg.position_y_2.data)))
            self.get_logger().info("I heard seg2's position_z: "+  (str(msg.position_z_2.data)))
            self.get_logger().info("I heard seg2's velocity_x: "+  (str(msg.velocity_x_2.data)))
            self.get_logger().info("I heard seg2's velocity_y: "+  (str(msg.velocity_y_2.data)))
            self.get_logger().info("I heard seg2's velocity_z: "+  (str(msg.velocity_z_2.data)))
            
            self.get_logger().info("I heard seg3's position_x: "+ (str(msg.position_x_3.data)))
            self.get_logger().info("I heard seg3's position_y: "+ (str(msg.position_y_3.data)))
            self.get_logger().info("I heard seg3's position_z: "+  (str(msg.position_z_3.data)))
            self.get_logger().info("I heard seg3's velocity_x: "+  (str(msg.velocity_x_3.data)))
            self.get_logger().info("I heard seg3's velocity_y: "+  (str(msg.velocity_y_3.data)))
            self.get_logger().info("I heard seg3's velocity_z: "+  (str(msg.velocity_z_3.data)))
            
            self.get_logger().info("I heard seg4's position_x: "+ (str(msg.position_x_4.data)))
            self.get_logger().info("I heard seg4's position_y: "+ (str(msg.position_y_4.data)))
            self.get_logger().info("I heard seg4's position_z: "+  (str(msg.position_z_4.data)))
            self.get_logger().info("I heard seg4's velocity_x: "+  (str(msg.velocity_x_4.data)))
            self.get_logger().info("I heard seg4's velocity_y: "+  (str(msg.velocity_y_4.data)))
            self.get_logger().info("I heard seg4's velocity_z: "+  (str(msg.velocity_z_4.data)))
            
            self.get_logger().info("I heard seg5's position_x: "+ (str(msg.position_x_5.data)))
            self.get_logger().info("I heard seg5's position_y: "+ (str(msg.position_y_5.data)))
            self.get_logger().info("I heard seg5's position_z: "+  (str(msg.position_z_5.data)))
            self.get_logger().info("I heard seg5's velocity_x: "+  (str(msg.velocity_x_5.data)))
            self.get_logger().info("I heard seg5's velocity_y: "+  (str(msg.velocity_y_5.data)))
            self.get_logger().info("I heard seg5's velocity_z: "+  (str(msg.velocity_z_5.data)))
            
            self.get_logger().info("I heard seg6's position_x: "+ (str(msg.position_x_6.data)))
            self.get_logger().info("I heard seg6's position_y: "+ (str(msg.position_y_6.data)))
            self.get_logger().info("I heard seg6's position_z: "+  (str(msg.position_z_6.data)))
            self.get_logger().info("I heard seg6's velocity_x: "+  (str(msg.velocity_x_6.data)))
            self.get_logger().info("I heard seg6's velocity_y: "+  (str(msg.velocity_y_6.data)))
            self.get_logger().info("I heard seg6's velocity_z: "+  (str(msg.velocity_z_6.data)))
    
        
    def listener_callback_control_input_change(self, msg):
        if self.print_params: 
            self.get_logger().info("I heard control points"+ (str(msg.current_sim_time)))
        
        if int(msg.current_sim_time) % 3 == 0 and self.count ==0 and (msg.current_sim_time)>3.0 :
                self.control_input, self.control_torque_dir  = self.sampleControlTorque()
                self.count = 1
                self.get_logger().info("CHANGING CONTROL TORQUE")
        if self.count==1 and msg.current_sim_time % 3>2.9:
                self.count =0

def main():
    rclpy.init(args=None)
    elastica_control = ElasticaControl()
    rclpy.spin(elastica_control)
    
if __name__== "__main__":
    main()
#############################################################################