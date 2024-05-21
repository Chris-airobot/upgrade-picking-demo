import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image

# Initialize CvBridge
bridge = CvBridge()

def depth_image_callback(msg):
    # Convert ROS Image message to OpenCV image
    depth_image = bridge.imgmsg_to_cv2(msg, "16UC1")
    
    # Normalize the depth image to fall within the 0-255 range for visualization
    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
    
    # Convert the normalized depth image to 8-bit (0-255)
    depth_image_8bit = depth_image_normalized.astype('uint8')
    
    # Apply a colormap to the depth image to improve visualization
    depth_image_colormap = cv2.applyColorMap(depth_image_8bit, cv2.COLORMAP_JET)
    
    # Display the depth image
    cv2.imshow("depth image", depth_image_colormap)
    cv2.waitKey(1)

# Initialize ROS node
rospy.init_node('depth_image_listener', anonymous=True)

# Subscribe to the depth image topic
rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_image_callback)

# Spin to keep the script running
rospy.spin()
