from copy import deepcopy
from kortex_driver.msg import JointSpeed


def format_speed(qd):
    joint_speeds = []
    
    for i in range(len(qd)):
        joint_speed = deepcopy(JointSpeed())
        joint_speed.joint_identifier = i
        joint_speed.value = qd[i]
        joint_speeds.append(joint_speed)
    
    return joint_speeds    