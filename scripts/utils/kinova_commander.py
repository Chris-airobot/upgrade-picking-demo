#!/usr/bin/python3
import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"

import sys
import rospy
import moveit_commander
from typing import Tuple, Union
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_msgs.msg import DisplayTrajectory, MoveItErrorCodes, RobotTrajectory
from geometry_msgs.msg import Pose, Point,Quaternion, PoseStamped, PoseArray
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
import random
from moveit_commander.conversions import list_to_pose
from kortex_driver.srv import Base_ClearFaults
# from tf2_geometry_msgs import PoseStamped
PlanTuple = Tuple[bool, RobotTrajectory, float, MoveItErrorCodes]

class KinovaCommander(object):
    def __init__(self):
        
        # Initialize moveit! package
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Group names of the robot, found it by running the actual robot
        self.arm_group_name = 'arm'
        self.gripper_group_name = 'gripper'
        
        # Robot’s kinematic model and the robot’s current joint states
        self.robot = moveit_commander.RobotCommander()

        # Robot’s internal understanding of the surrounding world

        self.scene = moveit_commander.PlanningSceneInterface("/kinova_gen3_lite")

        # Interfaces for planning groups (group of joints) to plan an execute motions
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_group_name, ns="/kinova_gen3_lite")
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_group_name, ns="/kinova_gen3_lite")

        # Ros publisher that is used to display trajectories in Rviz
        self.display_trajectory_publisher = rospy.Publisher("/kinova_gen3_lite" +'/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        
        
        clear_faults_full_name = '/kinova_gen3_lite/base/clear_faults'
        rospy.wait_for_service(clear_faults_full_name)
        self.clear_faults = rospy.ServiceProxy(clear_faults_full_name, Base_ClearFaults)

        
        
        
        
        
        
        
        
        self.init_scene()
        # self.init_pose()
        
        
    def move_gripper(self, value: float):
        '''
        Moves the gripper to the relative positons
        
        Args:
        value (float): a value from 0 to 1 to scale the gripper
        0 means close
        1 means open
        '''
        gripper_joint_names = rospy.get_param("/kinova_gen3_lite/gripper_joint_names", [])
        # We only need one joint name since the rest will respond to one of them
        gripper_joint_name = gripper_joint_names[0]
        
        # Corrspond to the actual joint in the robot
        gripper_joint = self.robot.get_joint(gripper_joint_name)
        
        # Get the bounds for calculating the gripper's desired position 
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        target = gripper_joint.move(value * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos)
        
        
        return target
    
    def init_scene(self):
        '''
        Adding a table under the robot to ensure it does not hit the table in reality
        '''
        table_size = [2, 2, 0.86]
        
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.position.x = 0  
        table_pose.pose.position.y = 0  
        table_pose.pose.position.z = -table_size[2]/2-0.050001 
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=table_size)

    
    
    
    
    def init_pose(self):
        '''
        Hard-coded home pose (for easy_handeye package) of the robot
        '''
        # home pose of the robot
        joint_values = self.arm_group.get_current_joint_values()
        joint_values[0] = -48 * pi / 180
        joint_values[1] = 38 * pi / 180
        joint_values[2] = 137 * pi / 180
        joint_values[3] = 94 * pi / 180
        joint_values[4] = 74 * pi / 180
        joint_values[5] = 39 * pi / 180
        # joint_values[0] = 0
        # joint_values[1] = 0
        # joint_values[2] = 0
        # joint_values[3] = 0
        # joint_values[4] = 0
        # joint_values[5] = 0        
        self.arm_group.go(joint_values, wait=True)
        # self.move_gripper(1)
        
    def get_cartesion_pose(self):
        '''
        Get the current pose and display it
        '''
        pose: PoseStamped = self.arm_group.get_current_pose()
        
        print(f'The cartesian pose is:')
        print(pose.pose)
        quat = pose.pose.orientation
        print(f'The rotations angles:{euler_from_quaternion([quat.w, quat.x, quat.y, quat.z])}')
        return pose
    

    def get_arm_joint_values(self):
        '''
        Get the current joint and display it
        '''
        joints = self.arm_group.get_current_joint_values()
        
        for i in range(len(joints)):
            print(f"- joint_{i+1}: {joints[i]}")
            
        return joints
    
    
    def reach_joint_angles(self, values):
        '''
        vertical pose of the robot
        '''
        joint_values = self.arm_group.get_current_joint_values()
        joint_values = values
        self.arm_group.go(joint_values, wait=True)
        

    
    def camera_calibration_pose(self):
        '''
        Hard-coded calibration pose (for easy_handeye package) of the robot 
        '''

        # Gripper Pose
        # self.move_gripper(0.85)
        # Calibration pose of the robot
        joint_values = self.arm_group.get_current_joint_values()
        joint_values[0] = -5 * pi/180
        joint_values[1] = -55 * pi / 180
        joint_values[2] = 111 * pi / 180
        joint_values[3] = -14 * pi / 180
        joint_values[4] = 118 * pi / 180
        joint_values[5] = 92 * pi / 180
        self.arm_group.go(joint_values, wait=True)
        
    def move(self, target):
        # self.arm_group.set_pose_target(target)
        # return True
        attempted = False

        self.arm_group.set_joint_value_target(target, True)
        plan_tuple: PlanTuple = self.arm_group.plan()
        plan = self.unpack_plan(plan_tuple)
        answer = input("Press Enter to proceed or q to replot")
        if answer != "q":
            attempted = self.arm_group.execute(plan, wait=True)

            
            
        return attempted
    
    
    
    
    
    def goto_pose(self, pose, velocity=1.0, group_name=None, wait=True):
        """
        Move to pose
        :param pose: Array position & orientation [x, y, z, qx, qy, qz, qw]
        :param velocity: Velocity (fraction of max) [0.0, 1.0]
        :param group_name: Move group (use current if None)
        :param wait: Wait for completion if True
        :return: Bool success
        """

        if type(pose) is list:
            pose = list_to_pose(pose)
            
            
            
        self.arm_group.set_max_velocity_scaling_factor(velocity)
        self.arm_group.set_pose_target(pose)
        success = self.arm_group.go(wait=wait)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    
    
    def goto_pose_cartesian(self, pose, velocity=1.0, group_name=None, wait=True):
        """
        Move to pose following a cartesian trajectory.
        :param pose: Array position & orientation [x, y, z, qx, qy, qz, qw]
        :param velocity: Velocity (fraction of max) [0.0, 1.0]
        :param group_name: Move group (use current if None)
        :param wait: Wait for completion if True
        :return: Bool success
        """

        if type(pose) is list:
            pose = list_to_pose(pose)

        self.arm_group.set_max_velocity_scaling_factor(velocity)
        (plan, fraction) = self.arm_group.compute_cartesian_path(
                                           [pose],   # waypoints to follow
                                           0.005,    # eef_step
                                           0.0)      # jump_threshold
        if fraction != 1.0:
            raise ValueError('Unable to plan entire path!')

        success = self.arm_group.execute(plan, wait=wait)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def goto_named_pose(self, pose_name, velocity=1.0, group_name=None, wait=True):
        """
        Move to named pos
        :param pose: Name of named pose
        :param velocity: Velocity (fraction of max) [0.0, 1.0]
        :param group_name: Move group (use current if None)
        :param wait: Wait for completion if True
        :return: Bool success
        """

        self.arm_group.set_max_velocity_scaling_factor(velocity)
        self.arm_group.set_named_target(pose_name)
        success = self.arm_group.go(wait=wait)
        self.arm_group.stop()
        return success
    
    
    


    def stop(self):
        """
        Stop the current movement.
        """
        # if self.arm_group:
        self.arm_group.stop()

    def recover(self):
        """
        Call the error reset action server.
        """
        try:
            self.clear_faults()
        except rospy.ServiceException:
            rospy.logerr("Failed to call ClearFaults")
            return False
        else:
            rospy.loginfo("Cleared the faults successfully")
            rospy.sleep(2.5)

        return True
    
    
    
    
    
    
    
    
    
    def unpack_plan(self, plan_tuple: PlanTuple) -> Union[RobotTrajectory, None]:
        """Function used to unpack the tuple returned when planning with move_group.
        This seems to be different than is was in ros melodic, so this function
        is needed to adapt the old code to the changes.

        Args:
            plan_tuple: A plan tuple containing the plan and other success data.

        Returns:
            If the planning was successful, a trajectory that can be directly used for
            visualization and motion. If unsuccessful, None is returned.
        """

        # plan_tuple[0] is the success boolean flag
        if plan_tuple[0]:
            return plan_tuple[1]  # The RobotTrajectory
        else:
            # If needed, the exact error code can be parsed from plan_tuple[3]
            return None
    
    

    def move_left(self):
        # graspsed = input("Hit Enter to continue")
        # if graspsed != "q":
            # move upwards
        pose: PoseStamped = self.arm_group.get_current_pose()
        pose.pose.position.z += 0.2
        self.arm_group.set_joint_value_target(pose, True)
        self.arm_group.go(wait=True)
        # move left
        # joint_values = self.arm_group.get_current_joint_values()
        # joint_values[0] += 60 * pi/180
        # self.arm_group.go(joint_values, wait=True)       
        joints = self.arm_group.get_current_pose()
        pose.pose.position.y += 0.6
        self.arm_group.set_joint_value_target(pose, True)
        self.arm_group.go(wait=True)

        

        # Go downwards
        pose: PoseStamped = self.arm_group.get_current_pose()
        pose.pose.position.z -= 0.1
        self.arm_group.set_joint_value_target(pose, True)
        self.arm_group.go(wait=True)

        
        # Release the gripper
        self.move_gripper(1)
        
        
        self.init_pose()
        
        
    def demo_move(self, left):
        
        # move upwards
        pose: PoseStamped = self.arm_group.get_current_pose()
        pose.pose.position.z += 0.2
        self.arm_group.set_joint_value_target(pose, True)
        self.arm_group.go(wait=True)
        
        rospy.sleep(1)
        # move left    
        pose: PoseStamped = self.arm_group.get_current_pose()
        
        quat = [0.005311705479287762, 0.7192909056869605, -0.6941433696979334, 0.027520194136891465]
        
        if left:
            # pose.pose.position.x = random.uniform(0.2, 0.5)
            # pose.pose.position.y = random.uniform(0.3, 0.5)
            pose.pose.position.y += 0.5
            pose.pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])
        self.arm_group.set_joint_value_target(pose, True)
        self.arm_group.go(wait=True)
        
        rospy.sleep(1)
        
        
        # Go downwards
        pose: PoseStamped = self.arm_group.get_current_pose()
        pose.pose.position.z -= 0.1
        self.arm_group.set_joint_value_target(pose, True)
        self.arm_group.go(wait=True)
        
        # Release the gripper
        self.move_gripper(1)
        
        
        self.init_pose()
        

# if __name__ == "__main__":
#     rospy.init_node("test")
#     robot = RobotInitialization()
#     robot.camera_calibration_pose()
#     # robot.init_pose()
    
#     rospy.logwarn(robot.get_cartesion_pose())