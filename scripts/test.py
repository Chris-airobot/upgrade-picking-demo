#! /usr/bin/env python3
import rospy
from kortex_driver.msg import BaseCyclic_Feedback
from ggcnn.srv import GraspPrediction
from sensor_msgs.msg import Image
import cv_bridge
bridge = cv_bridge.CvBridge()
import cv2


# ggcnn_srv = rospy.ServiceProxy('/ggcnn_service/predict', GraspPrediction)
# s = ggcnn_srv.call()
rospy.init_node("testjaja")
def depth_img_callback(self, msg):
    # Doing a rospy.wait_for_message is super slow, compared to just subscribing and keeping the newest one.

    curr_depth_img = bridge.imgmsg_to_cv2(msg)
    
    return curr_depth_img
   
   
   
x = rospy.wait_for_message('/camera/depth/image_rect_raw', Image)

cv2.imshow("depth image", bridge.imgmsg_to_cv2(x))
cv2.waitKey()