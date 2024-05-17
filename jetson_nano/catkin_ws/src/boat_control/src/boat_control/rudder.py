import rospy
import mavros.param
import json
import time

from mavros_msgs.msg import RCIn, OverrideRCIn
from mavros_msgs.srv import SetMode
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String
from enum import Enum

from boat_control import apm

DEBUG_MODE = True

node_name = "steering"
topic_rudder = "rudder"
topic_manoeuvre = 'manoeuvre'
 
RUDDER_POSITION_NEUTRAL = None          # servo values gained by calibration script
RUDDER_POSITION_MAX_PORTSIDE = None     # servo values gained by calibration script
RUDDER_POSITION_MAX_STARBORD = None     # servo values gained by calibration script

RUDDER_RANGE = None   # in degree

# manoeuvres:
FOLLOW_COURSE = 0
START_SAILING = 1

log_topic = "/logmsg"

class ServoType(Enum):
    RUDDER = 1
    SAIL = 3

    @property
    def rc_channel(self) -> int:
        return self.value


def valueToDegree(value) -> int:
    """
    Converts a given value provided by the rudder servo into a degree with respect to the keel line.

    Parameters:
    -----------
    value : a servo value as integer

    Return:
    -------
    Returns a degree as integer
    """

    global RUDDER_RANGE

    global RUDDER_POSITION_MAX_PORTSIDE
    global RUDDER_POSITION_MAX_STARBORD
    global RUDDER_POSITION_NEUTRAL

    if RUDDER_RANGE == None:
        print("RUDDER_RANGE not initilized!")
        return None

    if value == RUDDER_POSITION_NEUTRAL:
        return 0
    elif value <= RUDDER_POSITION_MAX_PORTSIDE:
        return -RUDDER_RANGE
    elif value >= RUDDER_POSITION_MAX_STARBORD:
        return RUDDER_RANGE
    elif value < RUDDER_POSITION_NEUTRAL and value > RUDDER_POSITION_MAX_PORTSIDE:
        return - (1-(value-RUDDER_POSITION_MAX_PORTSIDE)/(RUDDER_POSITION_NEUTRAL-RUDDER_POSITION_MAX_PORTSIDE))*RUDDER_RANGE
    elif value > RUDDER_POSITION_NEUTRAL and value < RUDDER_POSITION_MAX_STARBORD:
        return ((value-RUDDER_POSITION_NEUTRAL) / (RUDDER_POSITION_MAX_STARBORD - RUDDER_POSITION_NEUTRAL)) * RUDDER_RANGE
    else:
        return None # never be the case...


def degreeToServoValue(degree):
    """
    Converts a degree with respect to the keel line into the corresponding servo value.
    """

    global RUDDER_RANGE

    global RUDDER_POSITION_MAX_PORTSIDE
    global RUDDER_POSITION_MAX_STARBORD
    global RUDDER_POSITION_NEUTRAL

    if RUDDER_RANGE == None:
        print("RUDDER_RANGE not initilized!")
        return None
    
    if degree == 0:
        return RUDDER_POSITION_NEUTRAL
    elif degree <= -RUDDER_RANGE:
        return RUDDER_POSITION_MAX_PORTSIDE
    elif degree >= RUDDER_POSITION_MAX_STARBORD:
        return RUDDER_POSITION_MAX_STARBORD
    elif degree >  -RUDDER_RANGE  and degree < 0:
        return RUDDER_POSITION_NEUTRAL - (1+degree/RUDDER_RANGE)*(RUDDER_POSITION_NEUTRAL-RUDDER_POSITION_MAX_PORTSIDE)
    elif degree > 0 and degree < RUDDER_RANGE:
        return degree/RUDDER_RANGE * (RUDDER_POSITION_MAX_STARBORD-RUDDER_POSITION_NEUTRAL) + RUDDER_POSITION_NEUTRAL
    else:
        return None # never be the case...


def get_rudder_postion():
    """
    Returns the current position of the rudder read from the apm in degree.
    """

    target_servo = ServoType.RUDDER
    postion = apm.get_rc_channel(target_servo.rc_channel)

    return valueToDegree(postion)


def steer(degree):
    """
    Sets the rudder in respect to a given degree
    """

    value = degreeToServoValue(degree)

    apm.set_apm_mode("MANUAL")
    apm.set_parameter("TARGET_MODE", 3)

    target_servo = ServoType.RUDDER
    apm.set_rc_channel(target_servo.rc_channel, value)

    msg = "RUDDER: Target Value: %d | Actual Value: %d", value, get_rudder_postion()
    rospy.loginfo(msg)

    global log_publisher
    log_publisher.publish(String(msg))

def callback_steer(data):
    # callback function handling in coming data.
    rospy.loginfo("inside rudder callback...")
    rospy.loginfo(data)
    global log_publisher
    log_publisher.publish(String(data))

    # setting the rudder.
    steer(data.data)


def callback_manouvre(data):
    """
    A callback function deciding which manouvre has to be performed.
    """

    global log_publisher
    if data == START_SAILING:
        msg = "RUDDER: Manoeuvre: Start Sailing. Code = %d", data
        rospy.loginfo(msg)
        log_publisher.publish(String(msg))
        steer(RUDDER_POSITION_NEUTRAL)
    elif data == FOLLOW_COURSE:
        msg = "RUDDER: Manoeuvre: Follow Course. Code = %d", data
        rospy.loginfo(msg)
        log_publisher.publish(String(msg))
        pass
    else:
        msg = "RUDDER: Unknown Manoeuvre. Code = %d", data
        rospy.loginfo(msg)
        log_publisher.publish(String(msg))


def listen():
    """
    A listener listing to certain ROS topics
    """
    # Create a ROS node listening or steering commands
    rospy.Subscriber(topic_rudder, Int16, callback_steer)
    rospy.Subscriber(topic_manoeuvre, Int32, callback_manouvre)

    #Listen for Rudder_Setting_Command
    rospy.Subscriber('rudder_value', Int32, rudder_value_callback)

    global log_publisher
    global log_topic
    log_publisher = rospy.Publisher(log_topic, String, queue_size=1)

    rospy.loginfo("Rudder is listening to commands...")
    rospy.spin()


def rudder_value_callback(data):
    """
    A callback function passing on data to steering function.
    """
    global log_publisher
    msg1 = "rudder_value received:"
    msg2 = data.data
    rospy.loginfo(msg1)
    rospy.loginfo(msg2)
    log_publisher.publish(String(msg1))
    log_publisher.publish(String(msg2))
    steer(data.data)

def read_callibration_data():
    """
    Sets the parameter of the rudder in respect to the calibration file
    Return a tuple of form (rudder_left, rudder_mid, rudder_right)
    """

    calibration_file = 'calibration_data.json'

    with open(calibration_file, 'r') as json_data:
        data = json.load(json_data)
        # print(data)
        return data['RUDDER_MIN'], data['RUDDER_MID'], data['RUDDER_MAX']


def run_testing():

    """
    A visual self-test...
    """

    # move the rudder for testing purposes

    steer(valueToDegree(RUDDER_POSITION_NEUTRAL))
    time.sleep(0.5)

    ######## Test I: max positions ##########
    # move to max left
    steer(valueToDegree(RUDDER_POSITION_MAX_PORTSIDE))
    time.sleep(1)

    # move to max right
    steer(valueToDegree(RUDDER_POSITION_MAX_STARBORD))
    time.sleep(1)

    ####### Test II: stepwise from neutral to max left ####

    steer(valueToDegree(RUDDER_POSITION_NEUTRAL))
    time.sleep(0.5)




def init_rudder():
    """
    inits all required stuff
    """

    print("Starting rudder.py...")

    global RUDDER_POSITION_MAX_PORTSIDE
    global RUDDER_POSITION_MAX_STARBORD
    global RUDDER_POSITION_NEUTRAL

    rospy.init_node(node_name, anonymous=True)
    rospy.loginfo("read calibration data")

    RUDDER_POSITION_MAX_PORTSIDE, RUDDER_POSITION_NEUTRAL, RUDDER_POSITION_MAX_STARBORD = read_callibration_data()
    msg = f"Parameters of rudder:\nLEFT: {RUDDER_POSITION_MAX_PORTSIDE}\nNEUTRAL: {RUDDER_POSITION_NEUTRAL}\nRIGHT{RUDDER_POSITION_MAX_STARBORD}"
    rospy.loginfo(msg)

    global log_publisher
    log_publisher.publish(String(msg))

    if RUDDER_RANGE == None:
        print("RUDDER_RANGE not initilized!")


    if DEBUG_MODE:
        run_testing()

    steer(RUDDER_POSITION_NEUTRAL)
    listen()
