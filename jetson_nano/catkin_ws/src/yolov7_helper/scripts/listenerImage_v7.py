#!/home/nano/.pyenv/versions/3.6.15/bin/python

# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# OpenCV2 for saving an image
import cv2
import calendar
import time
import ros_numpy

def image_callback(msg):
    print("Received an image!")
    nump_img = ros_numpy.numpify(msg)
    nump_img = cv2.cvtColor(nump_img, cv2.COLOR_BGR2RGB)
    # Save your OpenCV2 image as a jpeg
    current_GMT = time.gmtime()
    time_stamp = calendar.timegm(current_GMT)
    cv2.imwrite('/home/nano/isp-2022/jetson_nano/catkin_ws/src/yolov7_helper/log/camera_image_'+ str(time_stamp) +'.jpeg', nump_img)

def main():
    rospy.init_node('listenerImage')
    # Define your image topic
    image_topic = "/yolov7/yolov7/visualization"
    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
