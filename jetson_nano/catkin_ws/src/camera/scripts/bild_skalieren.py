#!/usr/bin/env python
 
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback1(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    resized_image = cv2.resize(cv_image, (640, 480))
    img_pub = rospy.Publisher('/camera/resized_image', Image,queue_size=1)
    resized_image=bridge.cv2_to_imgmsg(resized_image,"bgr8")
    img_pub.publish(resized_image)


def main():
    rospy.init_node('camera_subscriber', anonymous=True)
    rospy.Subscriber('/camera/frame_both', Image, callback1)
    rospy.spin()

if __name__ == '__main__':
    main()