#!/usr/bin/env python3
#/home/nano/.pyenv/versions/3.8.16/bin/python

"""
ROS node to track objects using SORT TRACKER and YOLOv3 detector (darknet_ros)
Takes detected bounding boxes from darknet_ros and uses them to calculated tracked bounding boxes
Tracked objects and their ID are published to the sort_track node
No delay here
"""


import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from sort import sort 
from cv_bridge import CvBridge
import cv2
import time
import calendar
import ros_numpy
from sensor_msgs.msg import Image
from sort_track.msg import IntList
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose


def get_parameters():
	"""
	Gets the necessary parameters from .yaml file
	Returns tuple
	"""
	camera_topic = rospy.get_param("~camera_topic")
	detection_topic = rospy.get_param("~detection_topic")
	tracker_topic = rospy.get_param('~tracker_topic')
	cost_threhold = rospy.get_param('~cost_threhold')
	min_hits = rospy.get_param('~min_hits')
	max_age = rospy.get_param('~max_age')
	return (camera_topic, detection_topic, tracker_topic, cost_threhold, max_age, min_hits)


def callback_det(data):
	global detections
	global trackers
	global track
	global track_prev
	global idk_class
	
	detections = []
	trackers = []
	
	
	if yolov7:
		for i, x in enumerate(data.detections):
			nameid = x.results[0].id
			idk_class = nameid
			score = x.results[0].score
			centerx = x.bbox.center.x
			centery = x.bbox.center.y
			sizex = x.bbox.size_x
			sizey = x.bbox.size_y
			xmin = centerx-(sizex/2)
			xmax = centerx+(sizex/2)
			ymin = centery-(sizey/2)
			ymax = centery+(sizey/2)
			detections.append(np.array([xmin,ymin, xmax, ymax, round(score,2)]))
		detections = np.array(detections)
	else:
		for box in data.bounding_boxes:
			detections.append(np.array([box.xmin, box.ymin, box.xmax, box.ymax, round(box.probability,2)]))
			idk_class = box.Class
		detections = np.array(detections)
	#Call the tracker
	trackers = tracker.update(detections)
	trackers = np.array(trackers, dtype='int')
	track_prev = track
	track = trackers
	msg.data = track
	print(msg.data)
	update_tracked()

def update_tracked():
	print(track_prev)
	print(track)
	for i, x in enumerate(track):
		for j, y in enumerate(track_prev):
			if y[4] == x[4]:
				centerx = x[2] - x[0] #centerx
				centery = x[3] - x[1] #centery

				centerx_prev = y[2] - y[0] #centerx
				centery_prev = y[3] - y[1] #centery

				vecx = centerx - centerx_prev
				vecy = centery - centery_prev
				rospy.loginfo(rospy.get_caller_id() + 'I heard %s, %s, %s', vecx, vecy, x[4])
				with open('/home/nano/isp-2022/jetson_nano/catkin_ws/src/yolov7_helper/log_track/log.txt', "a") as f:
					f.write('Vector x: ' + str(vecx) + ' Vector y: ' + str(vecy) + ' Objekt_ID: ' + str(x[4]) + '\n' )


	
def callback_image(data):
	global detections
	global track
	#Display Image
	#bridge = CvBridge()
	cv_rgb = ros_numpy.numpify(data)
	#cv_rgb = bridge.imgmsg_to_cv2(data, "bgr8")
	#TO DO: FIND BETTER AND MORE ACCURATE WAY TO SHOW BOUNDING BOXES!!
	#Detection bounding box
	for i, x in enumerate(detections):
		cv2.rectangle(cv_rgb, (int(detections[i][0]), int(detections[i][1])), (int(detections[i][2]), int(detections[i][3])), (100, 255, 50), 1)
		cv2.putText(cv_rgb , str(idk_class), (int(detections[i][0]), int(detections[i][1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 255, 50), lineType=cv2.LINE_AA)	
	#Tracker bounding box
	for i, x in enumerate(detections):
		cv2.rectangle(cv_rgb, (track[i][0], track[i][1]), (track[i][2], track[i][3]), (255, 255, 255), 1)
		cv2.putText(cv_rgb , str(track[i][4]), (track[i][0], track[i][3]), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), lineType=cv2.LINE_AA)
	#cv2.imshow("YOLO+SORT", cv_rgb)
	cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
	current_GMT = time.gmtime()
	time_stamp = calendar.timegm(current_GMT)
	cv2.imwrite('/home/nano/isp-2022/jetson_nano/catkin_ws/src/yolov7_helper/log_track/camera_image_'+ str(time_stamp) +'.jpeg', cv_rgb)
	
	cv2.waitKey(3)


def main():
	global tracker
	global msg
	global yolov7
	global track
	track = []
	yolov7 = False
	msg = IntList()
	while not rospy.is_shutdown():
		#Initialize ROS node
		rospy.init_node('sort_tracker', anonymous=False)
		rate = rospy.Rate(10)
		# Get the parameters
		(camera_topic, detection_topic, tracker_topic, cost_threshold, max_age, min_hits) = get_parameters()
		tracker = sort.Sort(max_age=max_age, min_hits=min_hits) #create instance of the SORT tracker
		cost_threshold = cost_threshold
		#Subscribe to image topic
		image_sub = rospy.Subscriber(camera_topic,Image,callback_image)
		#Subscribe to darknet_ros to get BoundingBoxes from YOLOv3
		if yolov7:
			sub_detection = rospy.Subscriber('/yolov7/yolov7', Detection2DArray , callback_det)
		else:
			sub_detection = rospy.Subscriber(detection_topic, BoundingBoxes , callback_det)
		#Publish results of object tracking
		pub_trackers = rospy.Publisher(tracker_topic, IntList, queue_size=10)
		#print(msg) #Testing msg that is published
		pub_trackers.publish(msg)
		rate.sleep()
		rospy.spin()


if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass

