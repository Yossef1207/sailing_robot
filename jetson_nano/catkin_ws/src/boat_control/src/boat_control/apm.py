from typing import List
import random

import rospy
import mavros.param
from mavros_msgs.msg import RCIn, OverrideRCIn
from mavros_msgs.srv import SetMode

ROS_TIMEOUT_SEC = 5
SET_MODE_SERVICE_ADDR = "/mavros/set_mode"
MAV_MODE_MANUAL_ARMED = 192
RC_OVERRIDE_TOPIC = '/mavros/rc/override'
RC_IN_TOPIC = "/mavros/rc/in"


def get_parameter(param : str) -> int:
    return mavros.param.param_get(param)

def set_parameter(param :str, value: int):
    mavros.param.param_set(param, value)

def set_apm_mode(mode : str):
    # See http://wiki.ros.org/mavros#mavros.2FPlugins.Services-3
    rospy.wait_for_service(SET_MODE_SERVICE_ADDR, ROS_TIMEOUT_SEC)
    service_proxy = rospy.ServiceProxy(SET_MODE_SERVICE_ADDR, SetMode)
    service_proxy(MAV_MODE_MANUAL_ARMED, mode)

def get_rc_channels() -> List[int] :
    rc_in_msg = rospy.wait_for_message(RC_IN_TOPIC, RCIn, ROS_TIMEOUT_SEC)
    return list(rc_in_msg.channels)

def get_rc_channel(channel : int) -> int: 
    channel_index = channel - 1
    return get_rc_channels()[channel_index]
    

def set_rc_channel(channel : int, value : int):
    channel_index = channel - 1
    # Read current rc channel values
    #channels = get_rc_channels()
    channels = [0]*18
    # Override given rc channel
    channels[channel_index] = value
    # extend the channel list by 10, to match the size of the OverrideRCIn message
    #channels += ([0]*10)
    while rc_publisher.get_num_connections() < 1:
        # Wait for publisher to be ready
        pass
    rc_publisher.publish(channels=channels)


# Code to execute once (on first import)
mavros.set_namespace()
rc_publisher = rospy.Publisher(RC_OVERRIDE_TOPIC, OverrideRCIn, queue_size=10, latch=True)


if __name__ == "__main__":
    # Example application code
    rospy.init_node('apm', anonymous=True)
    set_apm_mode("MANUAL")

    print(get_parameter("RUDDER_MIN"))
    set_parameter("RUDDER_MIN", random.randint(500, 800))
    print(get_parameter("RUDDER_MIN"))

    print("Channels before:")
    print(get_rc_channels())

    set_rc_channel(0, random.randint(800, 1500))

    print("Channels after:")
    print(get_rc_channels())
