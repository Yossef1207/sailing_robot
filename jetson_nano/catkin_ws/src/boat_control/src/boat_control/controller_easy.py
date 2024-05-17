#!/usr/bin/env python
"""
This script acts as a navigator / conductor of all the important sailing modules.
Based on the sensor data, it decides which course to follow.

Degree values are encoded by an additional 0.
Example: 90° --> 900
"""

import rospy
import json
from std_msgs.msg import Int32
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Float32
from boat_control import apm

#import mavros.param
#from mavros_msgs.msg import RCIn, OverrideRCIn
#from mavros_msgs.srv import SetMode

#from boat_control import apm
import time

# ROS
node_name = "controller"

topic_rudder = "rudder"
topic_manoeuvre = 'manoeuvre'
log_topic = "/logmsg"
gps_angle_topic = "/mavros/setpoint_angle"
topic_wind_direction = "wind_data"

# manoeuvres:
manoeuvre_at_home = -1
manoeuvre_follow_course = 0     #
manoeuvres_start_sailing = 1
manoeuvres_jibe = 2             # Halse
manoeuvres_tack = 3             # Wende
manoeuvre_do_noting = 4
manoeuvre_beating = 5           # kreuzen

# publisher
manoeuvre_publisher = None
rudder_publisher = None

# Settings
CLOCKING = 250  # in ms

# for navigation
CURRENT_WIND_ANGLE = None         # angle in respect to boat
CURRENT_DESTINATION = None       # angle in respect to boat

WIND_DIRECTION = None
LEFT = "--LINKS--"
RIGHT = "--RECHTS--"

SAIL_ANGLE = 0  # SOLL_WINKEL Segel zu Boot
TARGET_APPARENT = 0  # SOLL_WINKEL scheinbarer WIND (Segel zu Wind)


def controll_manoeuvre():
    """
    This script will publish manoeuvre start_sailing, will wait 2 seconds
    and published then follow_course.
    Afterwards keyboard input can change the manoeuvre
    """

    manoeuvre_pub = rospy.Publisher('manoeuvre', Int32, queue_size=10)

    manoeuvre_pub.publish(manoeuvres_start_sailing)
    time.sleep(2)
    manoeuvre_pub.publish(manoeuvre_follow_course)
    time.sleep(2)

    i = ''
    while i != '99':
        i = input(
            "Please enter manoeuvre: 0-follow_course, 1-start_sailing, 99-quit\n")
        if (i == '0' or i == '1'):
            manoeuvre_pub.publish(i)
        time.sleep(1)


def __log__(msg):
    """
    Global logging.
    """

    msg = "CONTROLLER: " + msg
    rospy.loginfo(msg)
    log_publisher.publish(String(msg))


def evaluate_manoeuvres(wind, destination):
    """
    In respect to the given wind direction and the destination a decision is made,
    which manoeuvre should be performed next.

    Parameters:
    -----------

    wind        :   The direction of the wind in respect to the sail (i.e. mesured by the windsensor) in degree
    destination :   The angle between boat and the GPS point to be reached in degree.

    Return:
    -------

    The manoeuvre to be performed as an integer. The corresponding variables are stored as global variables.


    """

    global LEFT
    global RIGHT
    global SAIL_ANGLE
    global TARGET_APPARENT
    global WIND_DIRECTION
    global is_sailing

    # calculate Wind in respect to the boat!
    # TODO: Negativer Wert oder >360 --> Fehler
    wind_boat = wind - SAIL_ANGLE

    # error handling
    if wind_boat < 0 or wind_boat > 3600:
        return manoeuvre_do_noting

    if wind_boat < 1800 and wind_boat > 0:
        WIND_DIRECTION = RIGHT
    else:
        WIND_DIRECTION = LEFT

    if not is_sailing:
        is_sailing = True
        __log__(f"next manoeuvre: {manoeuvres_start_sailing}")
        return manoeuvres_start_sailing

    # convert in respect to wind
    if WIND_DIRECTION == LEFT:       # wind-left
        dest = (3600 - wind_boat) + destination

        if dest < 0:
            dest = 3600 + dest

    else:
        dest = wind_boat + (-1) * destination

        if dest < 0:
            dest = 3600 + dest

    # in respect to wind...
    if dest >= 450 and dest < 1800:
        __log__(f"next manoeuvre: {manoeuvre_follow_course}")
        return manoeuvre_follow_course

    elif (dest >= 1800 and dest < 2700):
        __log__(f"next manoeuvre: {manoeuvres_jibe}")
        return manoeuvres_jibe
    elif dest >= 2700 and dest < 3150:
        __log__(f"next manoeuvre: {manoeuvres_tack}")
        return manoeuvres_tack
    elif dest >= 3150 or dest < 450:
        __log__(f"next manoeuvre: {manoeuvre_beating}")
        return manoeuvre_beating
    else:
        __log__(f"next manoeuvre: UNKNOWN!!!")
        # should never reached...
        return manoeuvre_do_noting


def tick_tock():
    """
    For clocking.
    """

    global CLOCKING
    time.sleep(CLOCKING)


def navigate():
    """
    Evaluates current situation and decides what to do.
    Decisions are made, which manouvre has to be performed.
    In respect to that, orders are given to all other components.
    """

    global CURRENT_WIND_ANGLE
    global CURRENT_DESTINATION
    global manoeuvre_publisher
    global rudder_publisher
    global SAIL_ANGLE
    global TARGET_APPARENT

    __log__("inside navigate...")

    if CURRENT_DESTINATION == None or CURRENT_WIND_ANGLE == None:
        __log__("inside navigate:\t not initilized!!!")
        return

    manoeuvre = evaluate_manoeuvres(CURRENT_WIND_ANGLE, CURRENT_DESTINATION)

    if manoeuvre == manoeuvre_do_noting:
        manoeuvre_publisher.publish(manoeuvre_follow_course)  # for sail
        rudder_publisher.publish(0)

    manoeuvre_publisher.publish(manoeuvre)

    dest = CURRENT_DESTINATION  # from GPS
    wind = CURRENT_WIND_ANGLE
    sail_open = 300

    # constant to open the sail by some degree

    if manoeuvre == manoeuvre_at_home:
        # We're at home, do nothing
        pass
    elif manoeuvre == manoeuvres_start_sailing:
        # Set Sail!

        if WIND_DIRECTION == LEFT:
            TARGET_APPARENT = 2700
            SAIL_ANGLE = 600
        else:
            TARGET_APPARENT = 900
            SAIL_ANGLE = -600

        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)
        time.sleep(8)

    elif manoeuvre == manoeuvre_follow_course:
        # Follow Course!

        # calculate SOLL-WINKEL
        if WIND_DIRECTION == LEFT:
            TARGET_APPARENT = 2700 + sail_open
            old_sail_angle = SAIL_ANGLE
            if old_sail_angle + dest > 50:
                #we cannot steer by changing the sail_angle -> we also have to change the target_apparent
                SAIL_ANGLE = 50
                TARGET_APPARENT -= ((old_sail_angle + dest) - 50)
            else:
                SAIL_ANGLE += dest

        else:
            #WIND_DRECTION == RIGHT
            TARGET_APPARENT = 900 - sail_open
            old_sail_angle = SAIL_ANGLE
            if old_sail_angle + dest < -50:
                # we cannot steer by changing the sail_angle -> we also have to change the target_apparent
                SAIL_ANGLE = -50
                TARGET_APPARENT += -((old_sail_angle + dest) - 50)
            else:
                SAIL_ANGLE += dest

        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

    elif manoeuvre == manoeuvres_jibe:
        # Prepare to jibe!
        jibe_first_try()

    elif manoeuvre == manoeuvres_tack:
        # Prepare to tack!
        tack_first_try()
    elif manoeuvre == manoeuvre_beating:
        # 
        if WIND_DIRECTION == LEFT:
            SAIL_ANGLE = 300
            TARGET_APPARENT = 3450

            apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
            apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        else:
            # Wind from RIGHT
            SAIL_ANGLE = -300
            TARGET_APPARENT = 150

            apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
            apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)
    else:
        # invalid manoeuvre...
        __log__("invalid manouvre")


def jibe_first_try():
    """
    Perform jibe (Halse).

    1)  Abfallen
    2)  Halbwindkurs fahren
    
    """

    global SAIL_ANGLE
    global TARGET_APPARENT
    global WIND_DIRECTION
    global RIGHT
    global LEFT

    apparent_before = TARGET_APPARENT
    sail_angle_before = SAIL_ANGLE

    if WIND_DIRECTION == LEFT:

        # Abfallen
        __log__("inside jibe (LINKS):\t Abfallen")
        TARGET_APPARENT = 2700 - 250
        SAIL_ANGLE = 650
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(2)

        # Fahre Raumwindkurs (360° - 135° zum Wind)
        __log__("inside jibe (LINKS):\t Raumwindkurs")
        TARGET_APPARENT = 1050
        SAIL_ANGLE = -30
        WIND_DIRECTION = RIGHT
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(2)

        # Abbrechen, weitere Drehung wird automatisch gemacht (Modus Follow_Course)
        return

    else:
        #### WIND_DIRECTION from RIGHT ####

        # Abfallen
        __log__("inside jibe (RECHTS):\t Abfallen")

        TARGET_APPARENT = 1150
        SAIL_ANGLE = -650
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(2)

        # Fahre Raumwindkurs (135° zum Wind)

        __log__("inside jibe (RECHTS):\t Raumwindkurs ")

        TARGET_APPARENT = 2550
        SAIL_ANGLE = 30
        WIND_DIRECTION = LEFT
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(2)

        # Abbrechen, weitere Drehung wird automatisch gemacht (Modus Follow_Course)
        return


def tack_first_try():

    """
    Permform tack (Wende):

    1)  Steer into Wind
    2)  Move sail to other side, move further (45° to wind)
    3)  Half-Wind Course (Halbwindkurs)
    
    Parameter:
    ----------

    angle   :   Angle in degree in which the boat should point after tack (in repect to wind)
    """

    global SAIL_ANGLE
    global TARGET_APPARENT
    global WIND_DIRECTION
    global RIGHT
    global LEFT

    apparent_before = TARGET_APPARENT
    sail_angle_before = SAIL_ANGLE

    if WIND_DIRECTION == LEFT:

        # in Wind rein steuern
        __log__("inside tack (LEFT):\t in Wind reinsteuern...")

        TARGET_APPARENT = 3450
        SAIL_ANGLE = 150
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(1)

        # Hole Segel auf andere Seite, drehe weiter (45° zum Wind)
        __log__("inside tack (LEFT):\t 45° zum Wind...")

        TARGET_APPARENT = 450
        SAIL_ANGLE = - 150
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(1)

  
        # Fahre Halbwindkurs, um wieder in Fahrt zu kommen
        __log__("inside tack (LEFT):\t Halbwindkurs...")

        TARGET_APPARENT = 900
        SAIL_ANGLE = -300
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(3)

        # Abbrechen, weitere Drehung wird automatisch gemacht
        return

    else:
        # WIND_DIRECTION from RIGHT

        # in Wind rein steuern
        __log__("inside tack (RIGHT):\t in Wind reinsteuern...")

        TARGET_APPARENT = 150
        SAIL_ANGLE = -150
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(1)

        # Hole Segel auf andere Seite, drehe weiter (45° zum Wind)
        __log__("inside tack (RIGHT):\t 45° zum Wind...")

        TARGET_APPARENT = 3150 
        SAIL_ANGLE = 150
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(1)

        # Fahre Halbwindkurs, um wieder in Fahrt zu kommen
        __log__("inside tack (RIGHT):\t HAlbwindkurs")

        TARGET_APPARENT =  2700
        SAIL_ANGLE = 300
        WIND_DIRECTION = LEFT
        apm.set_parameter("SAIL_ANGLE", SAIL_ANGLE)
        apm.set_parameter("TARGET_APPARENT", TARGET_APPARENT)

        time.sleep(3)

        # Abbrechen, weitere Drehung wird automatisch gemacht
        return


def callback_wind(data):

    global CURRENT_WIND_ANGLE

    CURRENT_WIND_ANGLE = int(data.data)*10   # encode with additional 0

    # log
    msg = "received wind: " + str(CURRENT_WIND_ANGLE)
    __log__(msg)
    navigate()


def callback_GPS(data):
    global CURRENT_WIND_ANGLE
    global CURRENT_DESTINATION

    CURRENT_DESTINATION = data.data * 10  # encode with additional 0

    # log
    msg = "received angle: " + str(CURRENT_DESTINATION)
    __log__(msg)
    navigate()


def init():
    """
    Inittializes all important stuff...
    """

    rospy.init_node(node_name, anonymous=True)

    global log_publisher
    global topic_wind_direction
    global gps_angle_topic
    global rudder_publisher
    global manoeuvre_publisher
    global is_sailing

    is_sailing = False

    # publishers
    log_publisher = rospy.Publisher(log_topic, String, queue_size=1)
    rudder_publisher = rospy.Publisher(topic_rudder, Int16, queue_size=1)
    manoeuvre_publisher = rospy.Publisher(topic_manoeuvre, Int32, queue_size=1)

    rospy.Subscriber(topic_wind_direction, Float32, callback_wind)
    rospy.Subscriber(gps_angle_topic, Int32, callback_GPS)

    __log__("booted.")
    rospy.spin()

    # never reached
