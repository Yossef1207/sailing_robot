import rospy

from boat_control import apm
import numpy as np
import math

from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String

class GPSNode():
    def __init__(self):
        self.position = [] # Current position of the boat
        self.target = [] # Next target position
        self.heading = 0 # Heading of the boat
        self.angle = 0 # Required direction in degrees, 0 is forward, 180 is backward, clockwise positive, rounded to int
        self.gps_target_topic = "/mavros/setpoint_raw/target_global"
        self.gps_position_topic = "/mavros/global_position/global"
        self.gps_heading_topic = "/mavros/global_position/compass_hdg"
        self.gps_angle_topic = "/mavros/setpoint_angle"
        self.gps_log_topic = "/logmsg"
        print(
            "Make sure that MAVROS master node is running, e.g. roslaunch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 fcu_protocol:=v1.0"
        )
        rospy.init_node("gps", anonymous=True)
        self.gps_target_subscriber = rospy.Subscriber(
            self.gps_target_topic, GlobalPositionTarget, self.update_target
        )
        self.gps_position_subscriber = rospy.Subscriber(
            self.gps_position_topic, NavSatFix, self.update_position
        )
        self.gps_heading_subscriber = rospy.Subscriber(
            self.gps_heading_topic, Float64, self.update_heading
        )
        self.gps_angle_publisher = rospy.Publisher(
            self.gps_angle_topic, Int32, queue_size=1
        )
        self.gps_log_publisher = rospy.Publisher(
            self.gps_log_topic, String, queue_size=1
        )
        self.log("GPS initialization complete.")
        rospy.spin()

    # Update angle based on new target
    def update_target(self,target_vector:GlobalPositionTarget):
        self.target = target_vector
        self.log(f"Target updated: [{self.target.latitude},{self.target.longitude}]")
        self.calculate_angle()

    # Update angle based on new position
    def update_position(self,position_vector:NavSatFix):
        self.position = position_vector
        self.log(f"Position updated: [{self.position.latitude},{self.position.longitude}]")
        self.calculate_angle()

    # Update angle based on new heading
    def update_heading(self,heading:Float64):
        self.heading = heading.data
        self.log(f"Heading updated: {self.heading}")
        self.calculate_angle()

    def calculate_angle(self):
        # Calculate absolute angle
        absolute_angle = math.degrees(math.atan2(float(self.target.latitude)-float(self.position.latitude),float(self.target.longitude)-float(self.position.longitude)))
        self.log(f"New absolute_angle: {absolute_angle}")

        # Calculate angle in relation to heading
        self.angle = absolute_angle - float(self.heading)
        self.log(f"New angle: {absolute_angle}")

        # Convert angle to be between [-179,180] degrees
        self.angle -= math.ceil(self.angle / 360 - 0.5) * 360

        # Publish angle
        self.gps_angle_publisher.publish(int(self.angle))
        self.log(f"Published angle: {self.angle}")

    def log(self,message):
        print(message)
        rospy.loginfo(message)
        self.gps_log_publisher.publish(String(message))

def init_gps():
    gps = GPSNode()
    rospy.spin()
