#!/usr/bin/env python3

from mavros_msgs.msg import RCIn
from mavros.param import param_get
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from boat_control import apm
import os
import json

FILENAME = "calibration_data.json"
WIND_MIN = 0
WIND_MAX = 1024

log_topic = "/logmsg"

class WindSensorNode:
    def __init__(self):
        rospy.init_node('wind_sensor_node', anonymous=True)
        self.pub = rospy.Publisher('wind_data', Float32, queue_size=10)
        global log_topic
        #self.log_pub = rospy.Publisher(log_topic, String, queue_size=1)

        __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__)))
        file_path = os.path.join(__location__,  FILENAME)
        with open(file_path, 'r') as f:
            calib_data = json.loads(f.read())

        if calib_data["WIND_MIN"] is not None:
            WIND_MIN = calib_data["WIND_MIN"]
        rospy.loginfo("Wind_min: " + str(WIND_MIN))

        if calib_data["WIND_MAX"] is not None:
            WIND_MAX = calib_data["WIND_MAX"]
        rospy.loginfo("Wind_max: " + str(WIND_MAX))

    def run(self):
        while not rospy.is_shutdown():
            # Read wind sensor data here and publish as Float32 message
	   # adjust  timeout according to rate of RC channel values
          # rc_in_msg = rospy.wait_for_message("/mavros/rc/in", RCIn, timeout=1)
            wind_speed = apm.get_rc_channel(6)
            rospy.loginfo(wind_speed)
            #self.log_pub.publish(String(wind_speed))
            wind_converted = (wind_speed-900)*(360/(WIND_MAX-WIND_MIN)) - 60
            if wind_converted<0:
                wind_converted = 360 + wind_converted
            rospy.loginfo(str(wind_converted) + " degrees")
            #self.log_pub.publish(wind_converted)
            self.pub.publish(wind_converted)
            rospy.sleep(1)


if __name__ == '__main__':
    try:
        node = WindSensorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
