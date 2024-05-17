#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from darknet_ros_msgs.msg import BoundingBoxes

focal_length = 33
baseline = 65
image_width = 640  # Annahme: Bildbreite des Kamerabildes
threshold_shift = image_width // 4  # Schwellenwert für die Verschiebung (viertel Bildbreite)

object_size = 0  # Variable zur Speicherung der Objektgroesse

rospy.init_node('stereo_vision_node')

def coords_disp_from_subscriber(x, y):
    global object_size, image_width

    # Verschiebe das Objekt in die Mitte des Bildes, falls zu weit links oder rechts
    if x < threshold_shift or x > image_width - threshold_shift:
        x = image_width // 2

    if object_size > 0:
        disparity_x = ((focal_length * object_size) / (x - focal_length))
        disparity_y = ((focal_length * object_size) / (y - focal_length))
        disparity = (disparity_x + disparity_y) / 2
        distance = ((baseline * focal_length) / (disparity))
        rospy.loginfo('Distance to object: {:.2f} cm'.format(distance))


bridge = CvBridge()

def object_callback(msg):
    global object_size

    for box in msg.bounding_boxes:
        x = box.xmin + (box.xmax - box.xmin) // 2
        y = box.ymin + (box.ymax - box.ymin) // 2
        object_size = box.xmax - box.xmin

        # Ueberpruefe, ob das Objekt zu weit links oder rechts im Bild ist
        if x < threshold_shift or x > image_width - threshold_shift:
            rospy.logwarn('Object is too far left or right in the image. Shifting to the center.')
            x = image_width // 2

        coords_disp_from_subscriber(x, y)

rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, object_callback)

rospy.spin()
