#!/usr/bin/env python3
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from swift import Swift
from roboticstoolbox import *
from kortex_driver.srv import *
from kortex_driver.msg import *
import spatialgeometry as sg
from math import pi
import rospy
from kortex_driver.msg import Base_JointSpeeds
from std_msgs.msg import Empty, Header
from robot_model import GEN3_LITE
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from util import *


class ROBOT_SETUP:
    def __init__(self):
    
            # Setup publisher
        try:
            rospy.init_node('robot_initialization')

            self.HOME_ACTION_IDENTIFIER = 2

            # Robot Parameter
            self.robot = GEN3_LITE()
            
            
            # Get node params
            self.robot_name = rospy.get_param('~robot_name', "kinova_gen3_lite")
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 6)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", True)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            # Init the movement publisher
            self.vel_pub = rospy.Publisher('/kinova_gen3_lite/in/joint_velocity', Base_JointSpeeds, queue_size=10)
            self.stop_pub = rospy.Publisher('/kinova_gen3_lite/in/stop', Empty, queue_size=1)
            self.clear_pub = rospy.Publisher('/kinova_gen3_lite/in/clear_faults', Empty, queue_size=1)

            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None

            
            ## Init the services
            
            # calling the clear fault service to make sure the robot does not have any fault states 
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            # service for getting the action 
            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

            # service for executing the action 
            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

            # Basically needed as long as when you want to use the cartesian path
            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)

            # Activate the gripper
            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

            # Receives the notification of every actions that you execute
            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
        
            # Get the actual product result of the robot, not gonna use it very often
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)

            # Validate the waypoints that you want to execute are valid
            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
        
        
        
    def cb_action_topic(self, notif: ActionNotification):
        self.last_action_notif_type = notif.action_event
        
    
    def move_to_pose(self, target: PoseStamped):
        # Create message type and populate
        vel_msg = Base_JointSpeeds()
        stop_msg = Empty()
        
        self.robot.q = self.robot.home
        
        arrived = False

        # Specify the gain for the p_servo method
        gain = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
        
        # Get the roll pitch yaw angles
        r,p,y = euler_from_quaternion([target.pose.orientation.w, 
                                      target.pose.orientation.x, 
                                      target.pose.orientation.y,
                                      target.pose.orientation.z])
        
        Tep = self.robot.fkine(self.robot.q) * sm.SE3.Trans(target.pose.position.x, 
                                                            target.pose.position.y, 
                                                            target.pose.position.z) * sm.SE3.RPY(r,p,y)
        Tep = Tep.A
        
        while not arrived:
            # Work out the base frame manipulator Jacobian using the current robot configuration
            J = self.robot.jacobe(self.robot.q)

            # The end-effector pose of the panda (using .A to get a numpy array instead of an SE3 object)
            Te = self.robot.fkine(self.robot.q).A

            # Since the Panda has 7 joints, the Jacobian is not square, therefore we must
            # use the pseudoinverse (the pinv method)
            J_pinv = np.linalg.inv(J)

            # Calculate the required end-effector velocity and whether the robot has arrived
            ev, arrived = rtb.p_servo(Te, Tep, gain=gain, threshold=0.001, method='rpy')
            # ev, arrived = rtb.p_servo(Te, Tep, gain=gain, threshold=0.001, method='angle-axis')

            # Calculate the required joint velocities and apply to the robot
            self.robot.qd = J_pinv @ ev

            vel_msg.joint_speeds = format_speed(self.robot.qd)
            self.vel_pub.publish(vel_msg)

        
        
        self.stop_pub.publish(stop_msg)
        
        
        
        
        
if __name__ == "__main__":
    demo = ROBOT_SETUP()
    target = PoseStamped()

    
    
    
    target = PoseStamped()
    target.header = Header(frame_id="base_link")
    target.pose.position.x = 0.1120190208231396
    target.pose.position.y = -0.031112418216675628
    target.pose.position.z = 0.19582804495254266
    target.pose.orientation.w = 0.028990420926175237
    target.pose.orientation.x = -0.7183905176398512
    target.pose.orientation.y = 0.692266977938245
    target.pose.orientation.z = -0.06197621250060151
    
    demo.move_to_pose(target)