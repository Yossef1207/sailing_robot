#!/usr/bin/env python3
 
# Import the necessary libraries
from pathlib import Path
import cv2  # OpenCV library
import rospy  # Python library for ROS
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image  # Image is the message type


def publish_message():
  # get calibration data from xml file
  calib_file = cv2.FileStorage()

  calib_file.open( str(Path(__file__).parent / 'stereoMap.xml'), cv2.FileStorage_READ)

  stereoMapL_x = calib_file.getNode('stereoMapL_x').mat()
  stereoMapL_y = calib_file.getNode('stereoMapL_y').mat()
  stereoMapR_x = calib_file.getNode('stereoMapR_x').mat()
  stereoMapR_y = calib_file.getNode('stereoMapR_y').mat()
 
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub_corr_both = rospy.Publisher('/camera/frame_corr_both', Image, queue_size=1)
  pub_both = rospy.Publisher('/camera/frame_both', Image, queue_size=1)

  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_pub_py', anonymous=True)
     
  # Go through the loop n times per second
  rate = rospy.Rate(rospy.get_param('sample_rate', 10))

  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  cap = cv2.VideoCapture(cv2.CAP_ANY)

  verbose = rospy.get_param('verbose', True)

  # While ROS is still running.
  while not rospy.is_shutdown():

      # Capture frame-by-frame
      ret, frame = cap.read()
      if ret:

        # Split image in left and right
        width = frame.shape[1]
        raw_left = frame[:, 0:(width//2)]
        raw_right = frame[:, width//2:]

        # apply calibration to both video parts
        right = cv2.remap(raw_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
        left = cv2.remap(raw_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT,0)
        frame_corr = np.hstack((left, right))

        # Publish the image.
        pub_both.publish(ros_numpy.msgify(Image, frame, encoding = "rgb8"))
        pub_corr_both.publish(ros_numpy.msgify(Image, frame_corr, encoding = "rgb8"))

        if verbose:
            rospy.loginfo('Publishing camera frames')

      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  #cv2.namedWindow('raw',cv2.WINDOW_NORMAL)
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
