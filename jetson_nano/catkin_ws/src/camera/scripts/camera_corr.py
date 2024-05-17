#!/usr/bin/env python3
 
# Import the necessary libraries
from pathlib import Path
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
import cv2, os # OpenCV library
import ros_numpy
import numpy as np

# get calibration data from xml file
calib_file = cv2.FileStorage()
home = Path.home()
calib_file.open(str(home) + "/isp-2022/jetson_nano/catkin_ws/src/camera/scripts/stereoMap.xml", cv2.FileStorage_READ)

stereoMapL_x = calib_file.getNode('stereoMapL_x').mat()
stereoMapL_y = calib_file.getNode('stereoMapL_y').mat()
stereoMapR_x = calib_file.getNode('stereoMapR_x').mat()
stereoMapR_y = calib_file.getNode('stereoMapR_y').mat()
    
print(stereoMapL_x)
print(stereoMapL_y)
print(stereoMapL_x)
print(stereoMapR_x)

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
  raw_left = current_frame[:, 0:(width//2)]
  raw_right = current_frame[:, width//2:]

  # apply calibration to both video parts
  right = cv2.remap(raw_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
  left = cv2.remap(raw_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT,0)
  
  current_frame[:, width//2, :] = 0
  print("frame size ", current_frame.shape)
  print("left ", left.shape)
  print("right ", right.shape)
 
  # Display video feed
  cv2.imshow("raw left", raw_left)
  cv2.imshow("raw right", raw_right)
  cv2.imshow("left", left)
  cv2.imshow("right", right)
  
  #ret, sol = cv2.(coeff, z, flags = cv2.DECOMP_QR)
  
  
  cv2.waitKey(50)
  
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('video_frames', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
	
  
  receive_message()

  
