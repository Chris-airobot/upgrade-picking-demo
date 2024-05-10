#! /usr/bin/env python3

from __future__ import division, print_function

import rospy

import os
import time
import datetime
from kortex_driver.msg import BaseCyclic_Feedback

from std_msgs.msg import Int16, Header
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
# from franka_msgs.msg import FrankaState, Errors as FrankaErrors

from  utils.kinova_commander import KinovaCommander

import helpers.transforms as tfh
from helpers.ros_control import ControlSwitcher

from ggcnn.msg import Grasp
from ggcnn.srv import GraspPrediction

from kinova_base_grasping_controller import Logger, Run, Experiment

Run.log_properties = ['success', 'time', 'quality']
Experiment.log_properties = ['success_rate', 'mpph']


class OpenLoopGraspController(object):
    """
    Perform open-loop grasps from a single viewpoint using the Panda robot.
    """
    def __init__(self):
        
        ggcnn_service_name = '/ggcnn_service'
        rospy.wait_for_service(ggcnn_service_name + '/predict')
        self.ggcnn_srv = rospy.ServiceProxy(ggcnn_service_name + '/predict', GraspPrediction)

        self.curr_velocity_publish_rate = 100.0  # Hz
        self.curr_velo_pub = rospy.Publisher('/cartesian_velocity_node_controller/cartesian_velocity', Twist, queue_size=1)
        self.max_velo = 0.10
        self.curr_velo = Twist()
        self.best_grasp = Grasp()

        # self.cs = ControlSwitcher({'moveit': 'position_joint_trajectory_controller',
        #                            'velocity': 'cartesian_velocity_node_controller'})
        # self.cs.switch_controller('moveit')
        
        self.kc = KinovaCommander()

        self.robot_state = None
        self.ROBOT_ERROR_DETECTED = False
        self.BAD_UPDATE = False
        rospy.Subscriber('/kinova_gen3_lite/base_feedback', BaseCyclic_Feedback , self.__robot_state_callback, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose_viz", PoseArray, queue_size=1)
        # Centre and above the bin
        self.pregrasp_pose = [(rospy.get_param('/grasp_entropy_node/histogram/bounds/x2') + rospy.get_param('/grasp_entropy_node/histogram/bounds/x1'))/2 - 0.03,
                              (rospy.get_param('/grasp_entropy_node/histogram/bounds/y2') + rospy.get_param('/grasp_entropy_node/histogram/bounds/y1'))/2 + 0.10,
                              rospy.get_param('/grasp_entropy_node/height/z1') + 0.05,
                              2**0.5/2, -2**0.5/2, 0, 0]

        self.last_weight = 0
        # self.__weight_increase_check()

        self.experiment = Experiment()

    def __recover_robot_from_error(self):
        rospy.logerr('Recovering')
        self.kc.recover()
        rospy.logerr('Done')
        self.ROBOT_ERROR_DETECTED = False

    def __weight_increase_check(self):
        try:
            w = rospy.wait_for_message('/scales/weight', Int16, timeout=2).data
            increased = w > self.last_weight
            self.last_weight = w
            return increased
        except:
            return input('No weight. Success? [1/0]') == '1'

    def __robot_state_callback(self, msg: BaseCyclic_Feedback):
        self.robot_state = msg
        # if any(self.robot_state.base.fault_bank_a or self.robot_state.base.fault_bank_b):
        #     self.stop()
        #     if not self.ROBOT_ERROR_DETECTED:
        #         rospy.logerr('Robot Error Detected')
        #     self.ROBOT_ERROR_DETECTED = True
        # for s in FrankaErrors.__slots__:
        #     if getattr(msg.current_errors, s):
        #         self.stop()
        #         if not self.ROBOT_ERROR_DETECTED:
        #             rospy.logerr('Robot Error Detected')
        #         self.ROBOT_ERROR_DETECTED = True

    def __execute_best_grasp(self):
            # self.cs.switch_controller('moveit')

            ret = self.ggcnn_srv.call()
            if not ret.success:
                return False
            best_grasp = ret.best_grasp
            self.best_grasp = best_grasp

            tfh.publish_pose_as_transform(best_grasp.pose, 'base_link', 'G', 0.5)
            print(f'Your pose is: {best_grasp.pose}')
            
            


                
                
                
            grasp_pose_stamped = PoseStamped(
                pose=best_grasp.pose, header=Header(frame_id="base_link")
            )
            self.pose_pub.publish(
                PoseArray(header=Header(frame_id="base_link"), poses=[best_grasp.pose])
            )
            self.kc.move(target=grasp_pose_stamped)
            
            
            
            
            
            
            
            
            
            if input('Continue?') == '0':
                return False

            # Offset for initial pose.
            initial_offset = 0.10
            LINK_EE_OFFSET = 0.138

            # Add some limits, plus a starting offset.
            best_grasp.pose.position.z = max(best_grasp.pose.position.z - 0.01, 0.026)  # 0.021 = collision with ground
            best_grasp.pose.position.z += initial_offset + LINK_EE_OFFSET  # Offset from end efector position to

            self.kc.move_gripper(best_grasp.width)
            rospy.sleep(0.1)
            self.kc.goto_pose(best_grasp.pose, velocity=0.1)

            # Reset the position
            best_grasp.pose.position.z -= initial_offset + LINK_EE_OFFSET

            # self.cs.switch_controller('velocity')
            v = Twist()
            v.linear.z = -0.05

            # Monitor robot state and descend
            while self.robot_state.O_T_EE[-2] > best_grasp.pose.position.z and not any(self.robot_state.cartesian_contact) and not self.ROBOT_ERROR_DETECTED:
                self.curr_velo_pub.publish(v)
                rospy.sleep(0.01)
            v.linear.z = 0
            self.curr_velo_pub.publish(v)

            # Check for collisions
            if self.ROBOT_ERROR_DETECTED:
                return False

            # close the fingers.
            rospy.sleep(0.2)
            self.kc.grasp(0, force=2)

            # Sometimes triggered by closing on something that pushes the robot
            if self.ROBOT_ERROR_DETECTED:
                return False

            return True

    def stop(self):
        self.kc.stop()
        self.curr_velo = Twist()
        self.curr_velo_pub.publish(self.curr_velo)

    def go(self):
        input('Press Enter to Start.')
        while not rospy.is_shutdown():
            # self.cs.switch_controller('moveit')
            # self.kc.goto_named_pose('grip_ready', velocity=0.25)
            self.kc.init_pose()
            self.kc.goto_pose(self.pregrasp_pose, velocity=0.5)
            rospy.sleep(0.5)
            self.kc.move_gripper(1)

            # self.cs.switch_controller('velocity')

            run = self.experiment.new_run()
            run.start()
            grasp_ret = self.__execute_best_grasp()
            run.stop()

            if not grasp_ret or self.ROBOT_ERROR_DETECTED:
                rospy.logerr('Something went wrong, aborting this run')
                break

            # Release Object
            # self.cs.switch_controller('moveit')
            # self.kc.goto_named_pose('grip_ready', velocity=0.5)
            self.kc.init_pose()
            
            # self.kc.goto_named_pose('drop_box', velocity=0.5)
            self.kc.move_gripper(1)

            # Check success using the scales.
            rospy.sleep(1.0)
            grasp_success = self.__weight_increase_check()
            if not grasp_success:
                rospy.logerr("Failed Grasp")
            else:
                rospy.logerr("Successful Grasp")

            run.success = grasp_success
            run.quality = self.best_grasp.quality
            run.save()


if __name__ == '__main__':
    rospy.init_node('open_loop_grasp')
    pg = OpenLoopGraspController()
    pg.go()