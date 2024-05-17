#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes

focal_length = 33  # Fokallaenge der Kamera (in Pixeln)
baseline = 65  # Baseline-Abstand zwischen den Kameras

rospy.init_node('stereo_vision_node')

object_size = 0

def coords_disp_from_subscriber(x_left, y_left, x_right, y_right):
    # Berechne die Disparitaet basierend auf den Koordinaten des Objekts in beiden Kameras
    disparity_x = abs((focal_length * object_size) / (x_left - x_right))
    disparity_y = abs((focal_length * object_size) / (y_left - y_right))
    disparity = (disparity_x + disparity_y) / 2

    # Berechne die Entfernung durch Triangulation
    distance = (baseline * focal_length) / (disparity)
    rospy.loginfo('Distance to object: {:.2f} cm'.format(distance))

def object_callback():#msg):
    global object_size
    left_x = None
    left_y = None
    right_x = None
    right_y = None
    left_x = 251 #box.xmin + (box.xmax - box.xmin) // 2
    left_y = 351 #box.ymin + (box.ymax - box.ymin) // 2
    right_x = 250 #box.xmin + (box.xmax - box.xmin) // 2
    right_y = 350 #box.ymin + (box.ymax - box.ymin) // 2
    object_size =  80 #box.xmax - box.xmin
    coords_disp_from_subscriber(left_x, left_y, right_x, right_y)

    #for box in msg.bounding_boxes:
       # if box.id == 0:  # Annahme, dass 'id=0' die Bounding Box der linken Kamera ist
           # left_x = 10 #box.xmin + (box.xmax - box.xmin) // 2
            #left_y = 20 #box.ymin + (box.ymax - box.ymin) // 2
        #elif box.id == 1:  # Annahme, dass 'id=1' die Bounding Box der rechten Kamera ist
            #right_x = 13 #box.xmin + (box.xmax - box.xmin) // 2
            #right_y = 22 #box.ymin + (box.ymax - box.ymin) // 2
        #object_size =  30 #box.xmax - box.xmin

    #if left_x is not None and right_x is not None:
        #coords_disp_from_subscriber(left_x, left_y, right_x, right_y)

#rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, object_callback)
object_callback()

rospy.spin()
