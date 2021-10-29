import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import Bool


##########ROS2######################################################
class MinimalPublisherSubscriberForces(Node):
    
    def __init__(self, sim_params, rod_state, print_params):
        super().__init__('minimal_publisher_subscriber_forces')
        self.print_params = print_params
        self.t_coeff_optimized  = sim_params["t_coeff_optimized"] 
        self.period = sim_params["period"]
        self.wave_length = sim_params["wave_length"]
        self.base_length = sim_params["base_length"]
        self.wave_number = sim_params["wave_number"]
        self.phase_shift = sim_params["phase_shift"]
        self.rest_lengths = sim_params["rest_lengths"]
        self.ramp_up_time_MuscleTorques = sim_params["ramp_up_time_MuscleTorques"]
        self.direction_MuscleTorques = sim_params["direction_MuscleTorques"]
        self.with_spline = sim_params["with_spline"]
        self.muscle_torque_mag = sim_params["muscle_torque_mag"]
        self.force = sim_params["force"]
        self.direction_UniformForces = sim_params["direction_UniformForces"]
        self.uniformforces_mag = sim_params["uniformforces_mag"]
        self.torque = sim_params["torque"]
        self.direction_UniformTorques = sim_params["direction_UniformTorques"]
        self.uniformtorques_mag = sim_params["uniformtorques_mag"]
        self.start_force = sim_params["start_force"]
        self.end_force = sim_params["end_force"]
        self.ramp_up_time_EndpointForces = sim_params["ramp_up_time_EndpointForces"]
        self.acc_gravity = sim_params["acc_gravity"]
        self.dynamic_viscosity = sim_params["dynamic_viscosity"]
        self.start_force_mag = sim_params["start_force_mag"]
        self.end_force_mag = sim_params["end_force_mag"]
        self.ramp_up_time_EndpointForcesSinusoidal = sim_params["ramp_up_time_EndpointForcesSinusoidal"]
        self.tangent_direction = sim_params["tangent_direction"]
        self.normal_direction = sim_params["normal_direction"]
        self.rod_tip_orientation = rod_state["rod_tip_orientation"]
        self.position_x = rod_state["position_x"]
        self.position_y = rod_state["position_y"]
        self.position_z = rod_state["position_z"]
        self.velocity_x = rod_state["velocity_x"]
        self.velocity_y = rod_state["velocity_y"]
        self.velocity_z = rod_state["velocity_z"]
        
       
        self.publisher0 = self.create_publisher(Float64MultiArray, '/t_coeff_optimized', 10)
        self.publisher1 = self.create_publisher(Float64, '/period', 10)
        self.publisher2 = self.create_publisher(Float64, '/wave_length', 10)
        self.publisher3 = self.create_publisher(Float64, '/base_length', 10)
        self.publisher4 = self.create_publisher(Float64, '/wave_number', 10)
        self.publisher5 = self.create_publisher(Float64, '/phase_shift', 10)
        self.publisher6 = self.create_publisher(Float64MultiArray, '/rest_lengths', 10)
        self.publisher7 = self.create_publisher(Float64, '/ramp_up_time_MuscleTorques', 10)
        self.publisher8 = self.create_publisher(Float64MultiArray, '/direction_MuscleTorques', 10)
        self.publisher9 = self.create_publisher(Bool, '/with_spline', 10)
        self.publisher10 = self.create_publisher(Float64MultiArray, '/muscle_torque_mag', 10)
        self.publisher11 = self.create_publisher(Float64, '/force', 10)
        self.publisher12 = self.create_publisher(Float64MultiArray, '/direction_UniformForces', 10)
        self.publisher13 = self.create_publisher(Float64, '/torque', 10)
        self.publisher14 = self.create_publisher(Float64MultiArray, '/direction_UniformTorques', 10)
        self.publisher15 = self.create_publisher(Float64MultiArray, '/start_force', 10)
        self.publisher16 = self.create_publisher(Float64MultiArray, '/end_force', 10)
        self.publisher17 = self.create_publisher(Float64, '/ramp_up_time_EndpointForces', 10)
        self.publisher18 = self.create_publisher(Float64MultiArray, '/acc_gravity', 10)
        self.publisher19 = self.create_publisher(Float64, '/dynamic_viscosity', 10)
        self.publisher20 = self.create_publisher(Float64, '/start_force_mag', 10)
        self.publisher21 = self.create_publisher(Float64, '/end_force_mag', 10)
        self.publisher22 = self.create_publisher(Float64, '/ramp_up_time_EndpointForcesSinusoidal', 10)
        self.publisher23 = self.create_publisher(Float64MultiArray, '/tangent_direction', 10)
        self.publisher24 = self.create_publisher(Float64MultiArray, '/normal_direction', 10)
        self.publisher25 = self.create_publisher(Float64MultiArray, '/uniformforces_mag', 10)
        self.publisher26 = self.create_publisher(Float64MultiArray, '/uniformtorques_mag', 10)
        self.publisher27 = self.create_publisher(Float64MultiArray, '/rod_tip_orientation', 10)
        self.publisher28 = self.create_publisher(Float64MultiArray, '/position_x', 10)
        self.publisher29 = self.create_publisher(Float64MultiArray, '/position_y', 10)
        self.publisher30 = self.create_publisher(Float64MultiArray, '/position_z', 10)
        self.publisher31 = self.create_publisher(Float64MultiArray, '/velocity_x', 10)
        self.publisher32 = self.create_publisher(Float64MultiArray, '/velocity_y', 10)
        self.publisher33 = self.create_publisher(Float64MultiArray, '/velocity_z', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription0 = self.create_subscription(Float64MultiArray,'/t_coeff_optimized',self.listener_callback_t_coeff_optimized,10)
        self.subscription1 = self.create_subscription(Float64,'/period',self.listener_callback_period,10)
        self.subscription2 = self.create_subscription(Float64,'/wave_length',self.listener_callback_wave_length,10)
        self.subscription3 = self.create_subscription(Float64,'/base_length',self.listener_callback_base_length,10)
        self.subscription4 = self.create_subscription(Float64,'/wave_number',self.listener_callback_wave_number,10)
        self.subscription5 = self.create_subscription(Float64,'/phase_shift',self.listener_callback_phase_shift,10)
        self.subscription6 = self.create_subscription(Float64MultiArray,'/rest_lengths',self.listener_callback_rest_lengths,10)
        self.subscription7 = self.create_subscription(Float64,'/ramp_up_time_MuscleTorques',self.listener_callback_ramp_up_time_MuscleTorques,10)
        self.subscription8 = self.create_subscription(Float64MultiArray,'/direction_MuscleTorques',self.listener_callback_direction_MuscleTorques,10)
        self.subscription9 = self.create_subscription(Bool,'/with_spline',self.listener_callback_with_spline,10)
        self.subscription10 = self.create_subscription(Float64MultiArray,'/muscle_torque_mag',self.listener_callback_muscle_torque_mag,10)
        self.subscription11 = self.create_subscription(Float64,'/force',self.listener_callback_force,10)
        self.subscription12 = self.create_subscription(Float64MultiArray,'/direction_UniformForces',self.listener_callback_direction_UniformForces,10)
        self.subscription13 = self.create_subscription(Float64,'/torque',self.listener_callback_torque,10)
        self.subscription14 = self.create_subscription(Float64MultiArray,'/direction_UniformTorques',self.listener_callback_direction_UniformTorques,10)
        self.subscription15 = self.create_subscription(Float64MultiArray,'/start_force',self.listener_callback_start_force,10)
        self.subscription16 = self.create_subscription(Float64MultiArray,'/end_force',self.listener_callback_end_force,10)
        self.subscription17 = self.create_subscription(Float64,'/ramp_up_time_EndpointForces',self.listener_callback_ramp_up_time_EndpointForces,10)
        self.subscription18 = self.create_subscription(Float64MultiArray,'/acc_gravity',self.listener_callback_acc_gravity,10)
        self.subscription19 = self.create_subscription(Float64,'/dynamic_viscosity',self.listener_callback_dynamic_viscosity,10)
        self.subscription20 = self.create_subscription(Float64,'/start_force_mag',self.listener_callback_start_force_mag,10)
        self.subscription21 = self.create_subscription(Float64,'/end_force_mag',self.listener_callback_end_force_mag,10)
        self.subscription22 = self.create_subscription(Float64,'/ramp_up_time_EndpointForcesSinusoidal',self.listener_callback_ramp_up_time_EndpointForcesSinusoidal,10)
        self.subscription23 = self.create_subscription(Float64MultiArray,'/tangent_direction',self.listener_callback_tangent_direction,10)
        self.subscription24 = self.create_subscription(Float64MultiArray,'/normal_direction',self.listener_callback_normal_direction,10)
        self.subscription25 = self.create_subscription(Float64MultiArray,'/uniformforces_mag',self.listener_callback_uniformforces_mag,10)
        self.subscription26 = self.create_subscription(Float64MultiArray,'/uniformtorques_mag',self.listener_callback_uniformtorques_mag,10)
        self.subscription27 = self.create_subscription(Float64MultiArray,'/rod_tip_orientation',self.listener_callback_rod_tip_orientation,10)
        self.subscription28 = self.create_subscription(Float64MultiArray,'/position_x',self.listener_callback_position_x,10)
        self.subscription29 = self.create_subscription(Float64MultiArray,'/position_y',self.listener_callback_position_y,10)
        self.subscription30 = self.create_subscription(Float64MultiArray,'/position_z',self.listener_callback_position_z,10)
        self.subscription31 = self.create_subscription(Float64MultiArray,'/velocity_x',self.listener_callback_velocity_x,10)
        self.subscription32 = self.create_subscription(Float64MultiArray,'/velocity_y',self.listener_callback_velocity_y,10)
        self.subscription33 = self.create_subscription(Float64MultiArray,'/velocity_z',self.listener_callback_velocity_z,10)

        
        # prevent unused variable warning
        self.subscription0
        self.subscription1
        self.subscription2
        self.subscription3
        self.subscription4
        self.subscription5
        self.subscription6
        self.subscription7
        self.subscription8
        self.subscription9
        self.subscription10
        self.subscription11
        self.subscription12
        self.subscription13
        self.subscription14
        self.subscription15
        self.subscription16
        self.subscription17
        self.subscription18
        self.subscription19
        self.subscription20
        self.subscription21
        self.subscription22
        self.subscription23
        self.subscription24
        self.subscription25
        self.subscription26
        self.subscription27
        self.subscription28
        self.subscription29
        self.subscription30
        self.subscription31
        self.subscription32
        self.subscription33


    def timer_callback(self):
        msg0 = Float64MultiArray()
        msg1 = Float64()
        msg2 = Float64()
        msg3 = Float64()
        msg4 = Float64()
        msg5 = Float64()
        msg6 = Float64MultiArray()
        msg7 = Float64()
        msg8 = Float64MultiArray()
        msg9 = Bool()
        msg10 = Float64MultiArray()
        msg11 = Float64()
        msg12 = Float64MultiArray()
        msg13 = Float64()
        msg14 = Float64MultiArray()
        msg15 = Float64MultiArray()
        msg16 = Float64MultiArray()
        msg17 = Float64()
        msg18 = Float64MultiArray()
        msg19 = Float64()
        msg20 = Float64()
        msg21 = Float64()
        msg22 = Float64()
        msg23 = Float64MultiArray()
        msg24 = Float64MultiArray()
        msg25 = Float64MultiArray()
        msg26 = Float64MultiArray()
        msg27 = Float64MultiArray()
        msg28 = Float64MultiArray()
        msg29 = Float64MultiArray()
        msg30 = Float64MultiArray()
        msg31 = Float64MultiArray()
        msg32 = Float64MultiArray()
        msg33 = Float64MultiArray()
        msg0.data = self.t_coeff_optimized.tolist()
        msg1.data = self.period
        msg2.data = self.wave_length
        msg3.data = self.base_length
        msg4.data = self.wave_number
        msg5.data = self.phase_shift
        msg6.data = self.rest_lengths.tolist()
        msg7.data = self.ramp_up_time_MuscleTorques
        msg8.data = self.direction_MuscleTorques.tolist()
        msg9.data = self.with_spline
        msg10.data = self.muscle_torque_mag.tolist()
        msg11.data = self.force
        msg12.data = self.direction_UniformForces.tolist()
        msg13.data = self.torque
        msg14.data = self.direction_UniformTorques.tolist()
        msg15.data = self.start_force.tolist()
        msg16.data = self.end_force.tolist()
        msg17.data = self.ramp_up_time_EndpointForces
        msg18.data = self.acc_gravity.tolist()
        msg19.data = self.dynamic_viscosity
        msg20.data = self.start_force_mag
        msg21.data = self.end_force_mag
        msg22.data = self.ramp_up_time_EndpointForcesSinusoidal
        msg23.data = self.tangent_direction.tolist()
        msg24.data = self.normal_direction.tolist()
        msg25.data = np.squeeze(self.uniformforces_mag).tolist()
        msg26.data = np.squeeze(self.uniformtorques_mag).tolist()
        msg27.data = np.squeeze(self.rod_tip_orientation).tolist()
        msg28.data = np.squeeze(self.position_x).tolist()
        msg29.data = np.squeeze(self.position_y).tolist()
        msg30.data = np.squeeze(self.position_z).tolist()
        msg31.data = np.squeeze(self.velocity_x).tolist()
        msg32.data = np.squeeze(self.velocity_y).tolist()
        msg33.data = np.squeeze(self.velocity_z).tolist()
        
        self.publisher0.publish(msg0)
        self.publisher1.publish(msg1)
        self.publisher2.publish(msg2)
        self.publisher3.publish(msg3)
        self.publisher4.publish(msg4)
        self.publisher5.publish(msg5)
        self.publisher6.publish(msg6)
        self.publisher7.publish(msg7)
        self.publisher8.publish(msg8)
        self.publisher9.publish(msg9)
        self.publisher10.publish(msg10)
        self.publisher11.publish(msg11)
        self.publisher12.publish(msg12)
        self.publisher13.publish(msg13)
        self.publisher14.publish(msg14)
        self.publisher15.publish(msg15)
        self.publisher16.publish(msg16)
        self.publisher17.publish(msg17)
        self.publisher18.publish(msg18)
        self.publisher19.publish(msg19)
        self.publisher20.publish(msg20)
        self.publisher21.publish(msg21)
        self.publisher22.publish(msg22)
        self.publisher23.publish(msg23)
        self.publisher24.publish(msg24)
        self.publisher25.publish(msg25)
        self.publisher26.publish(msg26)
        self.publisher27.publish(msg27)
        self.publisher28.publish(msg28)
        self.publisher29.publish(msg29)
        self.publisher30.publish(msg30)
        self.publisher31.publish(msg31)
        self.publisher32.publish(msg32)
        self.publisher33.publish(msg33)

    def listener_callback_t_coeff_optimized(self, msg):
        if self.print_params: print('I heard t_coeff_optimized:', msg.data)
    def listener_callback_period(self, msg):
        if self.print_params: print('I heard period:', msg.data)
    def listener_callback_wave_length(self, msg):
        if self.print_params: print('I heard wave_length:', msg.data)
    def listener_callback_base_length(self, msg):
        if self.print_params: print('I heard base_length:', msg.data)
    def listener_callback_wave_number(self, msg):
        if self.print_params: print('I heard wave_number:', msg.data)
    def listener_callback_phase_shift(self, msg):
        if self.print_params: print('I heard phase_shift:', msg.data)
    def listener_callback_rest_lengths(self, msg):
        if self.print_params: print('I heard rest_lengths:', msg.data)
    def listener_callback_ramp_up_time_MuscleTorques(self, msg):
        if self.print_params: print('I heard ramp_up_time_MuscleTorques:', msg.data)
    def listener_callback_direction_MuscleTorques(self, msg):
        if self.print_params: print('I heard direction_MuscleTorques:', msg.data)
    def listener_callback_with_spline(self, msg):
        if self.print_params: print('I heard with_spline:', msg.data)
    def listener_callback_muscle_torque_mag(self, msg):
        if self.print_params: print('I heard muscle_torque_mag:', msg.data)
    def listener_callback_force(self, msg):
        if self.print_params: print('I heard force:', msg.data)
    def listener_callback_direction_UniformForces(self, msg):
        if self.print_params: print('I heard direction_UniformForces:', msg.data)
    def listener_callback_torque(self, msg):         
        if self.print_params: print('I heard torque:', msg.data)
    def listener_callback_direction_UniformTorques(self, msg):
        if self.print_params: print('I heard direction_UniformTorques:', msg.data)
    def listener_callback_start_force(self, msg):
        if self.print_params: print('I heard start_force:', msg.data)
    def listener_callback_end_force(self, msg):
        if self.print_params: print('I heard end_force:', msg.data)
    def listener_callback_ramp_up_time_EndpointForces(self, msg):
        if self.print_params: print('I heard ramp_up_time_EndpointForces:', msg.data)
    def listener_callback_acc_gravity(self, msg):
        if self.print_params: print('I heard acc_gravity:', msg.data)
    def listener_callback_dynamic_viscosity(self, msg):  
        if self.print_params: print('I heard dynamic_viscosity:', msg.data)
    def listener_callback_start_force_mag(self, msg):
        if self.print_params: print('I heard start_force_mag:', msg.data)
    def listener_callback_end_force_mag(self, msg):
        if self.print_params: print('I heard end_force_mag:', msg.data)
    def listener_callback_ramp_up_time_EndpointForcesSinusoidal(self, msg):
        if self.print_params: print('I heard ramp_up_time_EndpointForcesSinusoidal:', msg.data)
    def listener_callback_tangent_direction(self, msg):
        if self.print_params: print('I heard tangent_direction:', msg.data)
    def listener_callback_normal_direction(self, msg):
        if self.print_params: print('I heard normal_direction:', msg.data)
    def listener_callback_uniformforces_mag(self, msg):
        if self.print_params: print('I heard uniformforces_mag:', msg.data)
    def listener_callback_uniformtorques_mag(self, msg):
        if self.print_params: print('I heard uniformtorques_mag:', msg.data)
    def listener_callback_rod_tip_orientation(self, msg):
        if self.print_params: print('I heard rod_tip_orientation:', msg.data)
    def listener_callback_position_x(self, msg):
        if self.print_params: print('I heard position_x:', msg.data)
    def listener_callback_position_y(self, msg):
        if self.print_params: print('I heard position_y:', msg.data)
    def listener_callback_position_z(self, msg):
        if self.print_params: print('I heard position_z:', msg.data)
    def listener_callback_velocity_x(self, msg):
        if self.print_params: print('I heard velocity_x:', msg.data)
    def listener_callback_velocity_y(self, msg):
        if self.print_params: print('I heard velocity_y:', msg.data)
    def listener_callback_velocity_z(self, msg):
        if self.print_params: print('I heard velocity_z:', msg.data)

#############################################################################