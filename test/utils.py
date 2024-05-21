import spatialmath as sm
import numpy as np
import transforms3d as t3d
from geometry_msgs.msg import Pose, Point, Quaternion
from scipy.spatial.transform import Rotation
import tf
import tf.transformations

def pose_to_se3(pose_msg: Pose):
    
    # Extract translation components (x, y, z)
    translation = np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z])
    
    # # Extract orientation components (quaternion)
    quaternion = np.array([pose_msg.orientation.x, pose_msg.orientation.y,
                           pose_msg.orientation.z, pose_msg.orientation.w])

    # Convert quaternion to euler angles
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # Create SE(3) object using translation and euler angles
    se3 = sm.SE3.Trans(translation[0], translation[1], translation[2]) * sm.SE3.RPY(euler[0], euler[1], euler[2])
    
    return se3




