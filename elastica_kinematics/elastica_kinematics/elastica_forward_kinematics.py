__doc__ = """
For finding the transformed point after the transformation matrix (rotation + translation)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from elastica_msgs.msg import *
from geometry_msgs.msg import PoseStamped


##########ROS2######################################################
class PccForwardKinematics(Node):
    
    def __init__(self):
        super().__init__('forward_kinematics_calc')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('queue_size', None),
                ('print_params', None),
                ('pub_frequency', None),
                ('topic_names', ['elastica/control_input','elastica/time_tracker','elastica/rods_state','elastica/objs_state','elastica/pcc_kinematic_states', 'elastica/pcc_transformed_poses'])
            ])
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.print_params = self.get_parameter('print_params').get_parameter_value().integer_value
        self.pub_frequency = self.get_parameter('pub_frequency').get_parameter_value().double_value
        self.topic_names = self.get_parameter('topic_names').get_parameter_value().string_array_value
        
        self.no_of_segments = 6 #Number of Pneumatic chambers in the Continuum robot arm
        self.no_of_objects = 1 #Number of objects simualted in ELastica
        self.count = 0
        
        self.coord_segs = np.empty((self.no_of_segments+1,4,1)) #point coord after rotating and trnaslating
        self.coord_segs[0] = [[0.0],[0.0],[0.0], [1.0]]
        self.quaternion_segs = np.empty((self.no_of_segments,4))
        self.transformations = np.empty((self.no_of_segments,4,4))

        self.pcc_trans_poses_msg = PccTransformedPoses()

        self.publisher_pcc_trans_poses  =  self.create_publisher(PccTransformedPoses, self.topic_names[5], self.queue_size)
        
        
        timer_period = 1/self.pub_frequency  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
        self.subscription_pcc_kin_state = self.create_subscription(PccKinematicStates, self.topic_names[4],self.listener_callback_pcc_kin_states,self.queue_size)
        
        # prevent unused variable warning
        self.subscription_pcc_kin_state
        
        
    def transformed_poses(self):
        """
        n transformed poses for n segmnets using PCC model's forward kinematics. 
        
        
        Returns
        transformed pose each for the joint cosserat rods

        -------
        """
        
        for i in range(self.no_of_segments):
            self.coord_segs[i+1] = np.matmul(self.transformations[i], self.coord_segs[i])
            
            rot_mat = self.transformations[i][:3,:3]
            qw = (np.sqrt(1 + rot_mat[0, 0] + rot_mat[1, 1] + rot_mat[2, 2]) / 2)
            qx = ((rot_mat[2, 1] - rot_mat[1, 2]) / (4 * qw))
            qy = ((rot_mat[0, 2] - rot_mat[2, 0]) / (4 * qw))
            qz = ((rot_mat[1, 0] - rot_mat[0, 1]) / (4 * qw))
            self.quaternion_segs[i] = [qw,qx,qy,qz]
        
    def timer_callback(self):

        self.transformed_poses()
        for seg in range(self.no_of_segments):
            trans_seg_pose = PoseStamped()
            trans_seg_pose.pose.position.x = np.squeeze(self.coord_segs[seg+1][0]).tolist()
            trans_seg_pose.pose.position.y = np.squeeze(self.coord_segs[seg+1][1]).tolist()
            trans_seg_pose.pose.position.z = np.squeeze(self.coord_segs[seg+1][2]).tolist()
            trans_seg_pose.pose.orientation.w = np.squeeze(self.quaternion_segs[seg][0]).tolist()
            trans_seg_pose.pose.orientation.x = np.squeeze(self.quaternion_segs[seg][1]).tolist()
            trans_seg_pose.pose.orientation.y = np.squeeze(self.quaternion_segs[seg][2]).tolist()
            trans_seg_pose.pose.orientation.z = np.squeeze(self.quaternion_segs[seg][3]).tolist()
            self.pcc_trans_poses_msg.poses.append(trans_seg_pose)
        
        self.publisher_pcc_trans_poses.publish(self.pcc_trans_poses_msg)
        
    def listener_callback_pcc_kin_states(self, msg):
  
        for i in range(self.no_of_segments):self.transformations[i] = np.array(msg.transformations[i].data).reshape((4,4))
            

def main():
    rclpy.init(args=None)
    pcc_forward_kinematics = PccForwardKinematics()
    rclpy.spin(pcc_forward_kinematics)
    
if __name__== "__main__":
    main()
#############################################################################
