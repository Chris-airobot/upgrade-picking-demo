# Borrowed and modified from the kinova-ros examples.

import rospy
from kortex_driver.msg import Finger, GripperMode
from kortex_driver.srv import SendGripperCommandRequest, SendGripperCommand
import time



def set_finger_positions(finger_positions):
    """Send a gripper goal to the action server."""
    
    send_gripper_command_full_name = '/kinova_gen3_lite/base/send_gripper_command'
    rospy.wait_for_service(send_gripper_command_full_name)
    send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

    # Close the gripper
    req = SendGripperCommandRequest()
    finger = Finger()
    finger.finger_identifier = 0
    finger.value = finger_positions
    req.input.gripper.finger.append(finger)
    req.input.mode = GripperMode.GRIPPER_POSITION

    rospy.loginfo("Sending the gripper command...")

    # Call the service 
    try:
        send_gripper_command(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to call SendGripperCommand")
        return False
    else:
        time.sleep(0.5)
        return True




