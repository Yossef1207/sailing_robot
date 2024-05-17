#!/usr/bin/env python

# Package importation
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from openpyxl import Workbook
from sklearn.preprocessing import normalize

# Filtering
kernel = np.ones((3,3), np.uint8)

def coords_mouse_disp(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        average = 0
        for u in range(-1, 2):
            for v in range(-1, 2):
                average += disp[y+u, x+v]
        average = average/9
	Distance = -593.97*average**(3) + 1506.8*average**(2) - 1373.1*average + 522.06
        Distance = np.around(Distance*0.01, decimals=2)
        rospy.loginfo('Distance: %s m', str(Distance))

rospy.init_node('stereo_vision_node')
bridge = CvBridge()

# ROS publishers and subscribers
image_pub = rospy.Publisher('stereo_image', Image, queue_size=1)
disparity_pub = rospy.Publisher('disparity', Image, queue_size=1)

left_image = None
right_image = None

def image_callback_left(msg):
    global left_image
    left_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    process_stereo_images()

def image_callback_right(msg):
    global right_image
    right_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    process_stereo_images()

def process_stereo_images():
    if left_image is not None and right_image is not None:
        gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

	# Stereo image processing
        disp = stereo.compute(gray_left, gray_right)
        disp = ((disp.astype(np.float32) / 16) - min_disp) / num_disp

        # Filtering the results with a closing filter
        closing = cv2.morphologyEx(disp, cv2.MORPH_CLOSE, kernel)

	# Color mapping
        dispc = (closing - closing.min()) * 255
        dispC = dispc.astype(np.uint8)
        disp_color = cv2.applyColorMap(dispC, cv2.COLORMAP_OCEAN)

        # Publish the stereo image and disparity map
        image_pub.publish(bridge.cv2_to_imgmsg(left_image, encoding='bgr8'))
        disparity_pub.publish(bridge.cv2_to_imgmsg(disp_color, encoding='bgr8'))

# ROS subscribers for the camera images
rospy.Subscriber('/camera1/image_raw', Image, image_callback_left)
rospy.Subscriber('/camera2/image_raw', Image, image_callback_right)

rospy.spin()