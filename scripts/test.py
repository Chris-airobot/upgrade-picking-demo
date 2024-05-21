#! /usr/bin/env python3
import rospy
from ggcnn.srv import GraspPrediction
from sensor_msgs.msg import Image
import cv_bridge
bridge = cv_bridge.CvBridge()
import cv2
import helpers.transforms as tfh
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header
from helpers.timeit import TimeIt
import numpy as np



def process_depth_image(depth, crop_size=670, out_size=300, return_mask=False, crop_y_offset=0):
    imh, imw = depth.shape

    with TimeIt('1'):
        # Crop.
        depth_crop = depth[(imh - crop_size) // 2 - crop_y_offset:(imh - crop_size) // 2 + crop_size - crop_y_offset,
                           (imw - crop_size) // 2 :(imw - crop_size) // 2 + crop_size ]
    # depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)

    # Inpaint
    # OpenCV inpainting does weird things at the border.
    with TimeIt('2'):
        depth_crop = cv2.copyMakeBorder(depth_crop, 1, 1, 1, 1, cv2.BORDER_DEFAULT)
        depth_nan_mask = np.isnan(depth_crop).astype(np.uint8)
        
        
        print(f"depth_nan_mask - unique values: {np.unique(depth_nan_mask)}")
    with TimeIt('3'):
        depth_crop[depth_nan_mask==1] = 0

    with TimeIt('4'):
        # Scale to keep as float, but has to be in bounds -1:1 to keep opencv happy.
        depth_scale = np.abs(depth_crop).max()
        depth_crop = depth_crop.astype(np.float32) / depth_scale  # Has to be float32, 64 not supported.

        with TimeIt('Inpainting'):
            depth_crop = cv2.inpaint(depth_crop, depth_nan_mask, 1, cv2.INPAINT_NS)

        # Back to original size and value range.
        depth_crop = depth_crop[1:-1, 1:-1]
        depth_crop = depth_crop * depth_scale

    with TimeIt('5'):
        # Resize
        depth_crop = cv2.resize(depth_crop, (out_size, out_size), cv2.INTER_AREA)

    if return_mask:
        with TimeIt('6'):
            depth_nan_mask = depth_nan_mask[1:-1, 1:-1]
            depth_nan_mask = cv2.resize(depth_nan_mask, (out_size, out_size), cv2.INTER_NEAREST)
        return depth_crop, depth_nan_mask
    else:
        return depth_crop

def depth_img_callback(msg):
    # Doing a rospy.wait_for_message is super slow, compared to just subscribing and keeping the newest one.
    curr_depth_img = bridge.imgmsg_to_cv2(msg)
    depth_crop, depth_nan_mask = process_depth_image(curr_depth_img,return_mask=True)
    cv2.imshow('Processed Depth Image', depth_crop)
    # depth_nan_mask_display = (depth_nan_mask * 255).astype(np.uint8)
    # cv2.imshow('NaN Mask', depth_nan_mask_display)
    cv2.waitKey(1)
    
rospy.init_node("testjaja")

# Subscribe to the depth image topic
rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_img_callback)

rospy.spin()

