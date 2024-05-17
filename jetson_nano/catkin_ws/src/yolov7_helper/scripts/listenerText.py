#!/home/nano/.pyenv/versions/3.6.15/bin/python

# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from darknet_ros_msgs.msg import BoundingBoxes

def callback(data):
    location = "/home/nano/isp-2022/jetson_nano/catkin_ws/src/yolov7_helper/log/text.txt"
    rospy.loginfo(rospy.get_caller_id() + 'Picture start\n')
    with open(location, "a") as f:
             f.write(rospy.get_caller_id() + 'Picture start\n')

    for i, x in enumerate(data.bounding_boxes):
        nameid = x.id
        class_name = x.Class
        score = x.probability
        x_min = x.xmin
        y_min = x.ymin
        x_max = x.xmax
        y_max = x.ymax
        with open(location, "a") as f:

             f.write('I heard ' +str(nameid)+ ' ' + str(class_name) + ' ' + str(score)+ ' ' + str(x_min)+ ' ' + str(y_min)+ ' ' + str(x_max)+ ' ' + str(y_max)+ '\n' )
        rospy.loginfo('I heard %s, %s, %s, %s, %s, %s, %s', nameid, class_name, score, x_min, y_min, x_max, y_max)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
   
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

