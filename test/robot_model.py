#!/usr/bin/python3
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
import numpy as np
from roboticstoolbox import *
from math import pi
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseArray
from kortex_driver.srv import *
from kortex_driver.msg import *
import spatialmath as sm
import roboticstoolbox as rtb
import time
import moveit_commander
import moveit_msgs.msg
from utils import *

class GEN3_LITE(ERobot):

    def __init__(self):

        # links, name, urdf_string, urdf_filepath = self.URDF_read("/home/riot/kinova_gen3_lite/src/ros_kortex/kortex_description/arms/gen3_lite/6dof/urdf/gen3_lite_macro.xacro")
        links, name, urdf_string, urdf_filepath = self.URDF_read("/home/riot/kinova_gen3_lite/src/ros_kortex/kortex_description/robots/gen3_lite_gen3_lite_2f.xacro")
    
        super().__init__(
            links,
            name='gen3-lite',
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            manufacturer="Kinova",
            gripper_links=links[7]
        )
        
    
        try:
            if rospy.get_param("/gazebo/time_step", None):
                self.is_simulation = True
            else:
                self.is_simulation = False
            
            
            self.HOME_ACTION_IDENTIFIER = 2

            # Get node params
            self.robot_name = "kinova_gen3_lite"
            self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 6)
            self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", True)

            rospy.loginfo("Using robot_name " + self.robot_name + " , robot has " + str(self.degrees_of_freedom) + " degrees of freedom and is_gripper_present is " + str(self.is_gripper_present))

            ##### Init the topics #####
            # Init the action topic subscriber
            self.action_topic_sub = rospy.Subscriber("/" + self.robot_name + "/action_topic", ActionNotification, self.cb_action_topic)
            self.last_action_notif_type = None
            
            # Pose visualisation topic in rviz
            self.pose_pub = rospy.Publisher("/pose_viz", PoseArray, queue_size=1)
            
    
            ##### Init the services #####
            # Reset or clear fault states, allowing the robotic arm to recover from errors and resume normal operation
            clear_faults_full_name = '/' + self.robot_name + '/base/clear_faults'
            rospy.wait_for_service(clear_faults_full_name)
            self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

            joint_vel_cmd_full_name = '/' + self.robot_name + '/base/send_joint_speeds_command'
            rospy.wait_for_service(joint_vel_cmd_full_name)
            self.joint_vel_cmd = rospy.ServiceProxy(joint_vel_cmd_full_name, SendJointSpeedsCommand)
            
            read_action_full_name = '/' + self.robot_name + '/base/read_action'
            rospy.wait_for_service(read_action_full_name)
            self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
            
            execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
            rospy.wait_for_service(execute_action_full_name)
            self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)
            
            set_cartesian_reference_frame_full_name = '/' + self.robot_name + '/control_config/set_cartesian_reference_frame'
            rospy.wait_for_service(set_cartesian_reference_frame_full_name)
            self.set_cartesian_reference_frame = rospy.ServiceProxy(set_cartesian_reference_frame_full_name, SetCartesianReferenceFrame)
            
            send_gripper_command_full_name = '/' + self.robot_name + '/base/send_gripper_command'
            rospy.wait_for_service(send_gripper_command_full_name)
            self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)
            
            activate_publishing_of_action_notification_full_name = '/' + self.robot_name + '/base/activate_publishing_of_action_topic'
            rospy.wait_for_service(activate_publishing_of_action_notification_full_name)
            self.activate_publishing_of_action_notification = rospy.ServiceProxy(activate_publishing_of_action_notification_full_name, OnNotificationActionTopic)
            
            get_product_configuration_full_name = '/' + self.robot_name + '/base/get_product_configuration'
            rospy.wait_for_service(get_product_configuration_full_name)
            self.get_product_configuration = rospy.ServiceProxy(get_product_configuration_full_name, GetProductConfiguration)
            
            validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
            rospy.wait_for_service(validate_waypoint_list_full_name)
            self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)
        
        
            self.subscribe_to_a_robot_notification()
        except:
            self.is_init_success = False
        else:
            self.is_init_success = True
        if not self.is_simulation:
            
                rospy.loginfo("------- You are now in Real Robot Case -------")
        
        else:
            # For simulation, kinova does not have the same function in real robots, so we use Moveit Setups
            
            moveit_commander.roscpp_initialize(sys.argv)
        
            # Group names of the robot, found it by running the actual robot
            self.arm_group_name = 'arm'
            self.gripper_group_name = 'gripper'
            
            # Robot’s kinematic model and the robot’s current joint states
            self.robot = moveit_commander.RobotCommander()

            # Robot’s internal understanding of the surrounding world
            self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
            # Interfaces for planning groups (group of joints) to plan an execute motions
            self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name, ns=rospy.get_namespace())
            self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name, ns=rospy.get_namespace())

            # Ros publisher that is used to display trajectories in Rviz
            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace()+'/move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
            
            
            
            # Used for reset the world 
            # rospy.wait_for_service('/gazebo/reset_world')
            
        
            rospy.loginfo("------- You are now in Simulation Robot Case -------")

        
        self.ee_to_tool = sm.SE3.Tz(0.13) * sm.SE3.Rz(1.571)
        
        self.home = np.array([0   * pi/180, 
                              -16 * pi/180, 
                              75  * pi/180, 
                              0   * pi/180, 
                              -60 * pi/180, 
                              0   * pi/180])
        
        self.vertical = np.zeros(6)
        
        
        self.pre = np.array([-48 * pi/180, 
                             38  * pi/180, 
                             137 * pi/180, 
                             94  * pi/180, 
                             74  * pi/180, 
                             39  * pi/180])
        
        
        
        self.addconfiguration(
            "vertical", np.array([0, 0, 0, 0, 0, 0])
            
        )
        
        self.addconfiguration(
            "home", np.array([0   * pi/180, 
                              -16 * pi/180, 
                              75  * pi/180, 
                              0   * pi/180, 
                              -60 * pi/180, 
                              0   * pi/180])
            
        )
        
        self.addconfiguration(
            "pre", np.array([-48 * pi/180, 
                             38  * pi/180, 
                             137 * pi/180, 
                             94  * pi/180, 
                             74  * pi/180, 
                             39  * pi/180])
            
        )
        
        
        
        self.qdlim = np.array(
            [1, 1, 1, 1, 1, 1.57]
        )
        
        
    def cb_action_topic(self, notif: ActionNotification):
        self.last_action_notif_type = notif.action_event
    
    
    
    
    def pose_visualize(self, pose):
        self.pose_pub.publish(
            PoseArray(header=Header(frame_id="base_link"), poses=[pose])
        )
    

    def get_current_joint_values(self):
        """
        Get the current joint values of the robot in radians

        Returns:
            joint values in radians
        """
        joints = np.zeros(self.degrees_of_freedom)
        robot_data: BaseCyclic_Feedback = rospy.wait_for_message("/kinova_gen3_lite/base_feedback", BaseCyclic_Feedback, 1)
        
        for i in range(0, self.degrees_of_freedom):
            joints[i] = robot_data.actuators[i].position * pi/180

        return joints
    
    def stop(self):
        zero_speeds = Base_JointSpeeds()

        for i in range(self.degrees_of_freedom):
            joint = JointSpeed()
            joint.joint_identifier = i
            joint.value = 0.0
            self.joint_vel_cmd(SendJointSpeedsCommandRequest(input=zero_speeds))
           
    
    
    
    def move_pose(self, pose):
        """Use velocity control to move the robot of a simple pose

        Args:
            pose: based on the /tool_frame
        """
        self.pose_visualize(pose)
        
        input("Check Rviz, is it possible to do the pose?")
        
        gain = np.ones(6)
        se3_pose = pose_to_se3(pose)
        
        
        
        arrived = False
        while not arrived:
            
            # Work out the base frame manipulator Jacobian using the current robot configuration
            J = self.jacob0(self.get_current_joint_values())
            
            # The /tool_frame pose of the robot 
            Te = self.fkine(self.get_current_joint_values()) * self.ee_to_tool
            
            # Calculate the jacobian of the robot
            J_inv = np.linalg.inv(J)
            
            # Use the robotic toolbox to find the desired joint velocities
            ev, arrived = rtb.p_servo(Te.A, se3_pose.A, gain=gain, threshold=0.001, method='angle-axis')
            desired_joint_velocities = J_inv @ ev

            # Create the speed message
            speeds = Base_JointSpeeds()
        
            for i in range(self.degrees_of_freedom):
                joint = JointSpeed()
                joint.joint_identifier = i
                joint.value = desired_joint_velocities[i]
                speeds.joint_speeds.append(joint)

            # publish it to the topic
            self.joint_vel_cmd(SendJointSpeedsCommandRequest(input=speeds))

        self.stop()

           
            
    def move_trajectories(self, angles):
        """Move robots based on joint angles (in degrees)

        Args:
            angles: six joint angles in degrees

        Returns:
            _description_
        """
        if not self.is_simulation:
            self.last_action_notif_type = None

            req = ExecuteActionRequest()

            trajectory = WaypointList()
            waypoint = Waypoint()
            angularWaypoint = AngularWaypoint()

            # Angles to send the arm 
            for i in range(self.degrees_of_freedom):
                angularWaypoint.angles.append(angles[i])

            # Each AngularWaypoint needs a duration and the global duration (from WaypointList) is disregarded. 
            # If you put something too small (for either global duration or AngularWaypoint duration), the trajectory will be rejected.
            angular_duration = 0
            angularWaypoint.duration = angular_duration

            # Initialize Waypoint and WaypointList
            waypoint.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
            trajectory.duration = 0
            trajectory.use_optimal_blending = False
            trajectory.waypoints.append(waypoint)

            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False

            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)
            MAX_ANGULAR_DURATION = 30

            while (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION) :
                angular_duration += 1
                trajectory.waypoints[0].oneof_type_of_waypoint.angular_waypoint[0].duration = angular_duration

                try:
                    res = self.validate_waypoint_list(trajectory)
                except rospy.ServiceException:
                    rospy.logerr("Failed to call ValidateWaypointList")
                    return False

                error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

            if (angular_duration == MAX_ANGULAR_DURATION) :
                # It should be possible to reach position within 30s
                # WaypointList is invalid (other error than angularWaypoint duration)
                rospy.loginfo("WaypointList is invalid")
                return False

            req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
            
            # Send the angles
            rospy.loginfo("Sending the robot vertical...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteWaypointjectory")
                return False
            else:
                return self.wait_for_action_end_or_abort()       
            
        else:
            joint_values = self.arm_group.get_current_joint_values()
            joint_values = angles
            self.arm_group.go(joint_values, wait=True)    
            
            
            
            
            
    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)
                
                
                
                
    def subscribe_to_a_robot_notification(self):
        # Activate the publishing of the ActionNotification
        req = OnNotificationActionTopicRequest()
        rospy.loginfo("Activating the action notifications...")
        try:
            self.activate_publishing_of_action_notification(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call OnNotificationActionTopic")
            return False
        else:
            rospy.loginfo("Successfully activated the Action Notifications!")

        rospy.sleep(1.0)
        return True
    
    
    
    
if __name__ == "__main__":

    rospy.init_node("robot_model")
    kinova_lite = GEN3_LITE()