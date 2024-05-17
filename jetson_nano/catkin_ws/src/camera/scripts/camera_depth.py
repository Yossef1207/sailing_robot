#!/usr/bin/env python3
 
# Import the necessary libraries
from inspect import currentframe

from cv2 import StereoBM
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
import cv2, os # OpenCV library
import ros_numpy
import numpy as np

def nothing(x):
  pass

def callback(data):
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  # Copy read-only buffer to new array
  current_frame = np.array(ros_numpy.numpify(data))

  # Split image in left and right
  width = current_frame.shape[1]
  left = current_frame[:, 0:(width//2)]
  right = current_frame[:, width//2:]

  # convert video streams to grayscale
  left_gs = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
  right_gs = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)

  current_frame[:, width//2, :] = 0
  print("frame size ", current_frame.shape)
  print("left ", left.shape)
  print("right ", right.shape)
   
  # get slider positions 
  numDisparities = cv2.getTrackbarPos('numDisparities','disp')*16
  blockSize = cv2.getTrackbarPos('blockSize','disp')*2 + 5
  minDisparity = cv2.getTrackbarPos('minDisparity','disp')
  
  # refresh stereo vars
  stereo.setNumDisparities(numDisparities)
  stereo.setBlockSize(blockSize)
  stereo.setMinDisparity(minDisparity)
  
  # calculate depth image and rescale it
  depth = stereo.compute(left_gs, right_gs)
  depth = depth.astype(np.float32)
  depth = (depth/16.0 - minDisparity)/numDisparities
  
  # Display video feed
  cv2.imshow("left", left_gs)
  cv2.imshow("right", right_gs)
  #cv2.imshow("camera", current_frame)
  cv2.imshow("disp", depth)
  
  #ret, sol = cv2.(coeff, z, flags = cv2.DECOMP_QR)
  
  cv2.waitKey(50)
  
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('camera_depth', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('camera/frame_corr_both', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
	
  # create and setup depth config window
  cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
  cv2.createTrackbar('numDisparities','disp',1,17,nothing)
  cv2.createTrackbar('blockSize','disp',5,50,nothing)
  cv2.createTrackbar('minDisparity','disp',5,25,nothing)
  
  # init stereo class
  stereo : StereoBM = cv2.StereoBM_create()
  try:
    receive_message()
  except rospy.ROSInterruptException:
    pass  
