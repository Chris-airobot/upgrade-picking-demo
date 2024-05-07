# Borrowed and modified from the kinova-ros examples.

import rospy
# import actionlib
import kortex_driver.msg
# import geometry_msgs.msg
from kortex_driver.srv import ExecuteActionRequest, ExecuteAction
# import std_msgs.msg
from tf.transformations import euler_from_quaternion

def move_to_position(position, orientation):
    """Send a cartesian goal to the action server."""
    
    execute_action_full_name = '/kinova_gen3_lite/base/execute_action'
    rospy.wait_for_service(execute_action_full_name)
    position_client = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)


    req = ExecuteActionRequest()
    
    my_cartesian_speed = kortex_driver.msg.CartesianSpeed()
    my_cartesian_speed.translation = 0.1 # m/s
    my_cartesian_speed.orientation = 15  # deg/s

    my_constrained_pose = kortex_driver.msg.ConstrainedPose()
    my_constrained_pose.constraint.oneof_type.speed.append(my_cartesian_speed)
    # orientation in wxyz
    eulers = euler_from_quaternion(orientation)

    my_constrained_pose.target_pose.x = position[0]
    my_constrained_pose.target_pose.y = position[1]
    my_constrained_pose.target_pose.z = position[2]
    my_constrained_pose.target_pose.theta_x = eulers[0]
    my_constrained_pose.target_pose.theta_y = eulers[1]
    my_constrained_pose.target_pose.theta_z = eulers[2]
    
    
    req.input.oneof_action_parameters.reach_pose[0] = my_constrained_pose
    
    try:
        position_client(req)
    except rospy.ServiceException:
        rospy.logerr("Failed to send the pose")
    else:
        rospy.loginfo("Waiting for pose 1 to finish...")

    
    

    
    