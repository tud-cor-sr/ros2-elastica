__doc__ = """
For creating publishers & subscribers of ros topics for various simulation parameters, applied force & torque magnitudes 
& state space of cosserat rods simulated
"""
import numpy as np
from rclpy.node import Node
from elastica_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Vector3



##########ROS2######################################################
class ElasticaPublisherSubscriber(Node):
    
    def __init__(self, sim_params, rod_state, time_tracker, control_input, objs_state, obj_ids):
        super().__init__('elastica_pub_sub')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('queue_size', None),
                ('print_params', None),
                ('pub_frequency', None),
                ('topic_names', ['elastica/control_input','elastica/time_tracker','elastica/rods_state','elastica/physical_params', 'elastica/objs_state','elastica/pcc_kinematic_states'])
            ])
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.print_params = self.get_parameter('print_params').get_parameter_value().integer_value
        self.pub_frequency = self.get_parameter('pub_frequency').get_parameter_value().double_value
        self.topic_names = self.get_parameter('topic_names').get_parameter_value().string_array_value
        
        self.sim_params = sim_params
        self.rod_state = rod_state
        self.count = 0
        self.control_input = control_input
        self.objs_state = objs_state
        self.obj_ids = obj_ids

        self.physical_params = PhysicalParams()
        self.all_rods_state_msg = RodsState()
        self.sim_time_msg = SimulationTime()

        self.all_objs_state_msg = ObjsState()
        

        self.publisher_phys_params = self.create_publisher(PhysicalParams, self.topic_names[3], self.queue_size)
        self.publisher_rods_state = self.create_publisher(RodsState, self.topic_names[2], self.queue_size)
        self.publisher_sim_time  =  self.create_publisher(SimulationTime, self.topic_names[1], self.queue_size)
        self.publisher_objs_state = self.create_publisher(ObjsState, self.topic_names[4], self.queue_size)
        
        
        self.time_tracker = time_tracker
        timer_period = 1/self.pub_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
        self.subscription_ctrl_inp = self.create_subscription(ControlInput,self.topic_names[0],self.listener_callback_control_input,self.queue_size)
        
        # prevent unused variable warning
        self.subscription_ctrl_inp

        self.all_objs_state_msg.num_objects = self.sim_params["no_of_objects"]
        self.all_objs_state_msg.obj_names = ','.join(self.obj_ids)
        self.all_rods_state_msg.num_rods = self.sim_params["no_of_segments"]
     
        self.physical_params.b_coeff.data  = self.sim_params["b_coeff"].tolist()
        self.physical_params.period = self.sim_params["period"]
        self.physical_params.wave_length = self.sim_params["wave_length"]
        self.physical_params.base_length = self.sim_params["base_length"]
        self.physical_params.phase_shift = self.sim_params["phase_shift"]
        self.physical_params.ramp_up_time_muscle_torques = self.sim_params["ramp_up_time_MuscleTorques"]
        self.physical_params.direction_of_rod_extension.x = self.sim_params["direction_of_rod_extension"].tolist()[0]
        self.physical_params.direction_of_rod_extension.y = self.sim_params["direction_of_rod_extension"].tolist()[1]
        self.physical_params.direction_of_rod_extension.z = self.sim_params["direction_of_rod_extension"].tolist()[2]
        self.physical_params.with_spline = self.sim_params["with_spline"] 
        self.physical_params.direction_uniform_forces.x = self.sim_params["direction_UniformForces"].tolist()[0]
        self.physical_params.direction_uniform_forces.y = self.sim_params["direction_UniformForces"].tolist()[1]
        self.physical_params.direction_uniform_forces.z = self.sim_params["direction_UniformForces"].tolist()[2]
        self.physical_params.ramp_up_time_endpoint_forces = self.sim_params["ramp_up_time_EndpointForces"]
        self.physical_params.acc_gravity.linear.x = self.sim_params["acc_gravity"].tolist()[0]
        self.physical_params.acc_gravity.linear.y = self.sim_params["acc_gravity"].tolist()[1]
        self.physical_params.acc_gravity.linear.z = self.sim_params["acc_gravity"].tolist()[2]
        self.physical_params.dynamic_viscosity = self.sim_params["dynamic_viscosity"]
        self.physical_params.ramp_up_time_endpoint_forces_sinusoidal = self.sim_params["ramp_up_time_EndpointForcesSinusoidal"]
        self.physical_params.tangent_direction.x = self.sim_params["tangent_direction"].tolist()[0]
        self.physical_params.tangent_direction.y = self.sim_params["tangent_direction"].tolist()[1]
        self.physical_params.tangent_direction.z = self.sim_params["tangent_direction"].tolist()[2]
        self.physical_params.normal_direction.x = self.sim_params["normal_direction"].tolist()[0]
        self.physical_params.normal_direction.y = self.sim_params["normal_direction"].tolist()[1]
        self.physical_params.normal_direction.z = self.sim_params["normal_direction"].tolist()[2]
    
    def timer_callback(self):
        
        self.physical_params.force = self.sim_params["force"]
        self.physical_params.uniformtorques_mag.data = (np.array(self.sim_params["uniformtorques_mag"]).ravel()).tolist()
        self.physical_params.start_force.force.x = self.sim_params["start_force"].tolist()[0]
        self.physical_params.start_force.force.y = self.sim_params["start_force"].tolist()[1]
        self.physical_params.start_force.force.z = self.sim_params["start_force"].tolist()[2]
        self.physical_params.end_force.force.x = self.sim_params["end_force"].tolist()[0]
        self.physical_params.end_force.force.y = self.sim_params["end_force"].tolist()[1]
        self.physical_params.end_force.force.z = self.sim_params["end_force"].tolist()[2]
        self.physical_params.start_force_mag = self.sim_params["start_force_mag"]
        self.physical_params.end_force_mag = self.sim_params["end_force_mag"]


        for seg in range(self.sim_params["no_of_segments"]):
            single_rod_state_msg = RodState()
            single_rod_state_msg.num_elements = self.sim_params["n_elem"]
            for elem in range(self.sim_params["n_elem"]+1):
                elem_pose = PoseStamped()
                elem_vel = Vector3()
                kappa = Vector3()
                elem_pose.pose.position.x = np.squeeze(self.rod_state[seg]["position_x"]).tolist()[elem]
                elem_pose.pose.position.y = np.squeeze(self.rod_state[seg]["position_y"]).tolist()[elem]
                elem_pose.pose.position.z = np.squeeze(self.rod_state[seg]["position_z"]).tolist()[elem]
                if elem < self.sim_params["n_elem"]:
                    elem_pose.pose.orientation.w = np.squeeze(self.rod_state[seg]["orientation_ww"]).tolist()[elem]
                    elem_pose.pose.orientation.x = np.squeeze(self.rod_state[seg]["orientation_xx"]).tolist()[elem]
                    elem_pose.pose.orientation.y = np.squeeze(self.rod_state[seg]["orientation_yy"]).tolist()[elem]
                    elem_pose.pose.orientation.z = np.squeeze(self.rod_state[seg]["orientation_zz"]).tolist()[elem]
                elem_vel.x = np.squeeze(self.rod_state[seg]["velocity_x"]).tolist()[elem]
                elem_vel.y = np.squeeze(self.rod_state[seg]["velocity_y"]).tolist()[elem]
                elem_vel.z = np.squeeze(self.rod_state[seg]["velocity_z"]).tolist()[elem]
                if elem < self.sim_params["n_elem"]-1:
                    kappa.x = np.squeeze(self.rod_state[seg]["kappa_vec_x"]).tolist()[elem]
                    kappa.y = np.squeeze(self.rod_state[seg]["kappa_vec_y"]).tolist()[elem]
                    kappa.z = np.squeeze(self.rod_state[seg]["kappa_vec_z"]).tolist()[elem]
                single_rod_state_msg.poses.append(elem_pose)
                single_rod_state_msg.velocities.append(elem_vel)
                single_rod_state_msg.kappa.append(kappa)
            single_rod_state_msg.lengths.data = self.rod_state[seg]["lengths"].tolist()
            single_rod_state_msg.normal_director_base.data = self.rod_state[seg]["normal_director_base"].tolist()
            single_rod_state_msg.normal_director_tip.data = self.rod_state[seg]["normal_director_tip"].tolist()
            self.all_rods_state_msg.rod_states.append(single_rod_state_msg)
        
        
        for obj in range(self.sim_params["no_of_objects"]):
            obj_pose = PoseStamped()
            obj_pose.pose.position.x, obj_pose.pose.position.y, \
            obj_pose.pose.position.z = np.squeeze(self.objs_state[obj][str(self.obj_ids[obj])+"_position"]).tolist()
            
            obj_pose.pose.orientation.w, obj_pose.pose.orientation.x, obj_pose.pose.orientation.y,\
            obj_pose.pose.orientation.z = np.squeeze(self.objs_state[obj][str(self.obj_ids[obj])+"_orientation_ww_xx_yy_zz"]).tolist()
            
            self.all_objs_state_msg.obj_poses.append(obj_pose)
        
        self.sim_time_msg.current_sim_time = self.time_tracker.value
        
        self.publisher_phys_params.publish(self.physical_params)
        self.publisher_rods_state.publish(self.all_rods_state_msg)
        self.publisher_sim_time.publish(self.sim_time_msg)
        self.publisher_objs_state.publish(self.all_objs_state_msg)
        
    def listener_callback_control_input(self, msg):
        if self.print_params: 
            self.get_logger().info("I heard control torques "+ (str(msg.control_torques.data)))
            self.get_logger().info("I heard control torques directions"+ (str(np.array(msg.control_torque_dir.data))))
        self.control_input["control_torque"][:] = msg.control_torques.data
        self.control_input["control_torque_dir"][:] = np.array(msg.control_torque_dir.data)
        
    
#############################################################################