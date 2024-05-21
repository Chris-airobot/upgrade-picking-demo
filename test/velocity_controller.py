import os
os.environ["ROS_NAMESPACE"] = "/kinova_gen3_lite"
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
from swift import Swift
from roboticstoolbox import *
import spatialgeometry as sg
from typing import Tuple
from math import pi, sqrt
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from kortex_driver.srv import SendJointSpeedsCommand,SendJointSpeedsCommandRequest
from kortex_driver.msg import BaseCyclic_Feedback, JointSpeed, Base_JointSpeeds
from tf.transformations import euler_from_quaternion
from robot_model import GEN3_LITE
from scipy.spatial.transform import Rotation

target = [0,0,0,0,0,1.57]
nDOF = 6
d2r = pi/180
goal_tolerance = 4.0e-2
vec_difference = 100




def compute_vector_norm(vec):
    # Calculate the squared sum of all elements in the vector
    squared_sum = sum(x ** 2 for x in vec)
    # Take the square root of the squared sum to get the norm
    norm = sqrt(squared_sum)
    return norm



def vec_difference_goal2Actual(target):
    global nDOF
    global d2r
    global kinova_lite
    
    robot_data: BaseCyclic_Feedback = rospy.wait_for_message("/kinova_gen3_lite/base_feedback", BaseCyclic_Feedback, 1)
    
    
    res = []

    for i in range(0, nDOF):
        movement = target[i] - robot_data.actuators[i].position * d2r 
        res.append(movement)
    
    return res








if __name__ == "__main__":

    rospy.init_node("velocity_control")
    # reset_world = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    kinova_lite = GEN3_LITE()
    # if kinova_lite.simulation:
    #     reset_world()
    # rospy.sleep(2)
    
    print("Move the robot to the pre-grasp pose")
    kinova_lite.move_trajectories(kinova_lite.pre)

    # Correct for tool_frame
    Tep = sm.SE3.Trans(0.3, 0.3, 0.1) * sm.SE3.RPY(0.0, 3.14, 1.57)
    # For end_effector_link
    # Tep = sm.SE3.Trans(0.3, 0.3, 0.25) * sm.SE3.RPY(0.0, 3.14, 3.14)
    
    # Tep = sm.SE3.Trans(0.3, 0.3, 0.25) 
    
    pose_translation = Tep.A[:3, 3]
    pose_rotation = Rotation.from_matrix(Tep.A[:3, :3]).as_quat()
    
    pose = Pose(
        position = Point(x=pose_translation[0], y=pose_translation[1], z=pose_translation[2]),
        orientation=Quaternion(x=pose_rotation[0], y=pose_rotation[1], z=pose_rotation[2], w=pose_rotation[3]),
    )
    
    kinova_lite.move_pose(pose)
 
