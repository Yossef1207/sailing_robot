#!/usr/bin/env python3
import cv2
import rospy
import ros_numpy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox

class Mediator:
    def __init__(self, cfg):
        self.yolo_pub = rospy.Publisher(
            '/camera/rgb/image_raw', Image, queue_size=1
        )
        self.cfg = cfg
        for n in ['frame_slice', 'scale_factor', 'calib', 'log_detection', 'thresh', 'verbose']:
            rospy.loginfo(f'{n}: {self.cfg[n]}')

    def mediate(self, data):
        frame = ros_numpy.numpify(data)
        img, encoding = self.transform_frame(frame)
        if self.cfg['verbose']:
            rospy.loginfo(
                f'Mediating video frame {frame.shape} to yolo: {img.shape}'
            )
        self.yolo_pub.publish(ros_numpy.msgify(Image, img, encoding=encoding))

    def transform_frame(self, frame):
        width = frame.shape[1]
        center = width//2
        fslice = frame[:, center:] if self.cfg['frame_slice'] == 'right' else frame[:, :center]
        fslice = cv2.cvtColor(fslice, cv2.COLOR_BGR2RGB)
        scale_factor = self.cfg['scale_factor']
        interpolation = cv2.INTER_AREA if scale_factor < 1 else cv2.INTER_LINEAR
        if scale_factor != 1:
            fslice = cv2.resize(
                fslice, None, fx=scale_factor,
                fy=scale_factor, interpolation=interpolation
            )
        return fslice, "rgb8"

    def log_detection(self, msg):
        for box in msg.bounding_boxes:
            if box.probability > self.cfg['thresh']:
                rospy.loginfo(f'{box.Class:7s}: {box.probability:2.0%} bbox[x:{box.xmin:4d}, y:{box.ymin:4d}, w:{box.xmax-box.xmin:4d}, h:{box.ymax-box.ymin:4d}]')

def run():
    rospy.init_node('yolo_mediator_py', log_level=rospy.INFO)

    mediator = Mediator({
        k: rospy.get_param(k)
        for k in ['frame_slice', 'scale_factor', 'calib', 'log_detection', 'thresh', 'verbose']
    })

    frame_both_topic = '/camera/frame_corr_both' if rospy.get_param(
        'calib') else '/camera/frame_both'
    rospy.Subscriber(frame_both_topic, Image, mediator.mediate)

    if mediator.cfg['log_detection']:
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, mediator.log_detection)

    rospy.spin()


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
