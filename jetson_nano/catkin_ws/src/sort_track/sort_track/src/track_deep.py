#!/usr/bin/env python3
#/home/nano/.pyenv/versions/3.8.16/bin/python

"""
ROS node to track objects using DEEP_SORT TRACKER and YOLOv3 detector (darknet_ros)
Takes detected bounding boxes from darknet_ros and uses them to calculated tracked bounding boxes
Tracked objects and their ID are published to the sort_track node
For this reason there is a little delay in the publishing of the image that I still didn't solve
"""
import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
from deep_sort.detection import Detection
from deep_sort import nn_matching
from deep_sort.tracker import Tracker
from deep_sort import generate_detections as gdet
from deep_sort import preprocessing as prep
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sort_track.msg import IntList
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D, ObjectHypothesisWithPose
import ros_numpy
import time
import calendar



def get_parameters():
	"""
	Gets the necessary parameters from .yaml file
	Returns tuple
	"""
	camera_topic = rospy.get_param("~camera_topic")
	detection_topic = rospy.get_param("~detection_topic")
	tracker_topic = rospy.get_param('~tracker_topic')
	return (camera_topic, detection_topic, tracker_topic)


def callback_det(data):
	#print("Image received")
	global detections
	global scores
	global class_id
	detections = []
	scores = []
	global yolov7
	if yolov7:
		for i, x in enumerate(data.detections):
			nameid = x.results[0].id
			class_id = nameid
			score = x.results[0].score
			centerx = x.bbox.center.x
			centery = x.bbox.center.y
			sizex = x.bbox.size_x
			sizey = x.bbox.size_y
			xmin = centerx-(sizex/2)
			xmax = centerx+(sizex/2)
			ymin = centery-(sizey/2)
			ymax = centery+(sizey/2)
			detections.append(np.array([xmin,ymin, xmax-xmin, ymax-ymin]))
			scores.append(float('%.2f' % round(score,2)))
		detections = np.array(detections)
	else:
		for box in data.bounding_boxes:
			class_id = box.Class
			detections.append(np.array([box.xmin, box.ymin, box.xmax-box.xmin, box.ymax-box.ymin]))
			scores.append(float('%.2f' % box.probability))
		detections = np.array(detections)


def callback_image(data):
	global track_whole
	global track_whole_prev
	#Display Image
	#bridge = CvBridge()
	#cv_rgb = bridge.imgmsg_to_cv2(data, "bgr8")
	cv_rgb = ros_numpy.numpify(data)
	#Features and detections
	features = encoder(cv_rgb, detections)
	detections_new = [Detection(bbox, score, feature) for bbox,score, feature in
                        zip(detections,scores, features)]
	# Run non-maxima suppression.
	boxes = np.array([d.tlwh for d in detections_new])
	scores_new = np.array([d.confidence for d in detections_new])
	indices = prep.non_max_suppression(boxes, 1.0 , scores_new)
	detections_new = [detections_new[i] for i in indices]
	# Call the tracker
	tracker.predict()
	tracker.update(detections_new)
	#Detecting bounding boxes
	for det in detections_new:
		bbox = det.to_tlbr()
		cv2.rectangle(cv_rgb,(int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(100,255,50), 1)
		cv2.putText(cv_rgb , str(class_id), (int(bbox[0]), int(bbox[1])), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100,255,50), lineType=cv2.LINE_AA)
	#Tracker bounding boxes
	track_whole_prev = track_whole
	track_whole = []
	for track in tracker.tracks:
		if not track.is_confirmed() or track.time_since_update > 1:
				continue
		bbox = track.to_tlbr()
		msg.data = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]), track.track_id]
		#print(msg.data)
		track_whole.append(msg.data)
		cv2.rectangle(cv_rgb, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])),(255,255,255), 1)
		cv2.putText(cv_rgb, str(track.track_id),(int(bbox[0]), int(bbox[3])),cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,0,255),2)
	#print(track_whole)
	#cv2.imshow("YOLO+SORT", cv_rgb)
	cv_rgb = cv2.cvtColor(cv_rgb, cv2.COLOR_BGR2RGB)
	current_GMT = time.gmtime()
	time_stamp = calendar.timegm(current_GMT)
	cv2.imwrite('/home/nano/isp-2022/jetson_nano/catkin_ws/src/yolov7_helper/log_deep/camera_image_'+ str(time_stamp) +'.jpeg', cv_rgb)
	update_tracked()
	pub_trackers_image.publish(ros_numpy.msgify(Image, cv_rgb, encoding = "rgb8"))
	cv2.waitKey(3)

def update_tracked():
	for i, x in enumerate(track_whole):
		for j, y in enumerate(track_whole_prev):
			if y[4] == x[4]:
				centerx = x[2] - x[0] #centerx
				centery = x[3] - x[1] #centery

				centerx_prev = y[2] - y[0] #centerx
				centery_prev = y[3] - y[1] #centery

				vecx = centerx - centerx_prev
				vecy = centery - centery_prev
				rospy.loginfo(rospy.get_caller_id() + 'I heard %s, %s, %s', vecx, vecy, x[4])
				with open('/home/nano/isp-2022/jetson_nano/catkin_ws/src/yolov7_helper/log_deep/log.txt', "a") as f:
					f.write('Vector x: ' + str(vecx) + ' Vector y: ' + str(vecy) + ' Objekt_ID: ' + str(x[4]) + '\n' )
				vector_movement = IntList()
				vector_movement.data = (vecx, vecy, x[4])
				pub_trackers.publish(vector_movement)

def main():
	global tracker
	global encoder
	global msg
	global track_whole
	global track_whole_prev
	global pub_trackers_image
	global pub_trackers
	track_whole = []
	track_whole_prev = []
	msg = IntList()
	max_cosine_distance = 0.2
	nn_budget = 100
	metric = nn_matching.NearestNeighborDistanceMetric("cosine", max_cosine_distance, nn_budget)
	tracker = Tracker(metric)
	model_filename = "/home/nano/isp-2022/jetson_nano/catkin_ws/src/sort_track/sort_track/src/deep_sort/mars-small128.pb" #Change it to your directory
	encoder = gdet.create_box_encoder(model_filename)
	#Initialize ROS node
	rospy.init_node('sort_tracker', anonymous=True)
	rate = rospy.Rate(10)
	# Get the parameters
	(camera_topic, detection_topic, tracker_topic) = get_parameters()
	#Subscribe to image topic
	image_sub = rospy.Subscriber(camera_topic,Image,callback_image)

	#Subscribe to darknet_ros to get BoundingBoxes from YOLOv3 or YOLOv7
	global yolov7
	yolov7 = False
	if yolov7:
		sub_detection = rospy.Subscriber('/yolov7/yolov7', Detection2DArray , callback_det)
	else:
		sub_detection = rospy.Subscriber(detection_topic, BoundingBoxes , callback_det)
	while not rospy.is_shutdown():
		#Publish results of object tracking
		pub_trackers = rospy.Publisher('/deepsort/vector', IntList, queue_size=10)
		pub_trackers_image = rospy.Publisher('/deepsort/image', Image, queue_size=1)
		#print(msg)
	
		rate.sleep()


if __name__ == '__main__':
	try :
		main()
	except rospy.ROSInterruptException:
		pass

