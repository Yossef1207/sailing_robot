#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import String

import mavros.param
from mavros_msgs.msg import RCIn, OverrideRCIn
from mavros_msgs.srv import SetMode

from enum import Enum

import json
import time

from boat_control import apm

# Werte auslesen
WIND_MIN = 0
WIND_MAX = 0
WIND_MID = 0

SAIL_MIN = 920
SAIL_MAX = 1470
SAIL_MID = 2020

WIND_MID_BACK = 0
WIND_MID_FRONT = 0
WIND_INTERVAL = 1055

MAX_VALUE_WIND = 2000
MAX_VALUE_SAIL_2 = 1200
MAX_VALUE_SAIL = 2400

SAIL_OPEN = 0.2

log_topic = "/logmsg"

#Initialisiere important values variable!
important_values = [0, 0, 0, 0, 0, 0]
important_values_dif = [0, 99999, 0, 0, 0, 0]
#wind-speed, wind-direction, speed, manoeuvre, rudder-position, Neigung(left-right)

sail_position = SAIL_MID

class ServoType(Enum):
    RUDDER = 1
    SAIL = 3

    @property
    def rc_channel(self) -> int:
        return self.value


####################### functions ###########################

def read_calibration_data():
    calibration_file = 'calibration_data.json'

    # JSON-Datei importieren
    with open(calibration_file, 'r') as file:
        data = json.load(file)

    return data['WIND_MIN'], data['WIND_MID'], data['WIND_MAX'], data['SAIL_MIN'], data['SAIL_MID'], data['SAIL_MAX']

def set_sail(position):

    global SAIL_MAX
    global SAIL_MIN

    apm.set_apm_mode("MANUAL")
    apm.set_parameter("TARGET_MODE", 3)

    target_servo = ServoType.SAIL

    if int(position) > SAIL_MAX:
        apm.set_rc_channel(target_servo.rc_channel, SAIL_MAX)
    elif int(position) < SAIL_MIN:
        apm.set_rc_channel(target_servo.rc_channel, SAIL_MIN)
    else:
        apm.set_rc_channel(target_servo.rc_channel, int(position))

    global log_pub
    msg = "set_sail to: %d | Actual Value: %d", int(position), get_sail_position()
    log_pub.publish(String(msg))
    rospy.loginfo(msg)


def get_sail_position() -> int:
    target_servo = ServoType.SAIL
    position = apm.get_rc_channel(target_servo.rc_channel)
    return position

###################### callback-functions ######################

def listen():
    #rospy.init_node('sail_setter', anonymous=True)
      # Abonniere Topics
    #rospy.Subscriber('wind_speed', Float32, self.wind_s_callback)  #wie übergebe ich mehrere daten an eine Funktion?
    rospy.Subscriber('wind_data', Float32, wind_callback_1st)
    #rospy.Subscriber('speed', Int32, self.speed_callback)
    rospy.Subscriber('manoeuvre', Int32, manoeuvre_callback)   #-> entscheidet welche Funktion gestartet wird
    rospy.Subscriber('sail_value', Int32, sail_value_callback)
    #rospy.Subscriber('rudder', Float32, self.rudder_callback)
    #rospy.Subscriber('neigung', Float32, self.neigung_callback)
    global log_topic
    global log_pub
    log_pub = rospy.Publisher(log_topic, String, queue_size=1)

    # Erstelle einen Publisher für das Topic "sail_position"
    #sail_position_pub = rospy.Publisher('sail_position', Int32, queue_size=10)
    msg = "sail is listening to commands"
    log_pub.publish(String(msg))
    rospy.loginfo(msg)
    rospy.spin()

def sail_value_callback(data):
    global log_pub
    msg1 = "sail_value received:"
    msg2 = data.data
    log_pub.publish(String(msg1))
    log_pub.publish(String(msg2))
    rospy.loginfo(msg1)
    rospy.loginfo(msg2)
    set_sail(data.data)

def wind_callback_1st(data):
    global important_values

    important_values[1] = int(data.data)

    global log_pub
    msg = "Wind_value received: "
    log_pub.publish(String(msg))
    log_pub.publish(String(important_values[1]))
    rospy.loginfo(msg)
    rospy.loginfo(important_values[1])

    follow_course_adapted()

def wind_d_callback(data):

    global important_values
    global important_values_dif

    print(data.data)
    #rospy.loginfo(f"wind_value received: {data.data}")
    #rospy.loginfo(data)

    if important_values_dif[1] != 99999:
        important_values_dif[1] = data.data - important_values_dif[1]
        important_values[1]= data.data

        #rospy.loginfo("wind_value: " + important_values[1])
        #rospy.loginfo("wind_diff: " + important_values_dif[1])
            
        manoeuvre = important_values[3]
        if manoeuvre == 0:
            follow_course_adapted()
        elif manoeuvre == 1:
            start_sailing_adapted()
        
    #handle case: the wind value is received the first time -> dif is set to 0
    else:
        important_values[1] = data.data
        important_values_dif[1] = 0

        #rospy.loginfo("wind_value: " + important_values[1])
        #rospy.loginfo("wind_diff: " + important_values_dif[1])

        manoeuvre = important_values[3]
        if manoeuvre == 0:
                # dif of 0 leads to 0 change
                #self.follow_course()
            do = 'nothing'
        elif manoeuvre == 1:
            start_sailing_adapted()
        else:
            global log_pub
            msg = "!!!!!manoeuvre wrong!!!!!"
            log_pub.publish(String(msg))
            rospy.loginfo(msg)


def manoeuvre_callback(data):
    important_values[4]= data.data


###################### sailing - functions ######################

def start_sailing_adapted():
    global log_pub
    msg = "start_sailing_adapted()"
    log_pub.publish(String(msg))
    rospy.loginfo(msg)

    global sail_position
    global important_values
    global important_values_dif
    global SAIL_MID
    global SAIL_MAX
    global SAIL_MIN
    global SAIL_OPEN

    # !!!!!IMPORTANT Sail has to be set to SAIL_MID!!!!!!
    wind_value = important_values[1]

    if wind_value < 1705 and wind_value > 1175:
        wind = 'RIGHT'

        percent_wind = 1 - ((wind_value - 900) / 1060 + 0.25)

        wind_90 = percent_wind - 0.25 - SAIL_OPEN

        if wind_90 == 0:
            do = 'nothing'
        elif wind_90 < 0:
            sail_position = SAIL_MID - wind_90 * MAX_VALUE_SAIL_2
        else:
            sail_position = SAIL_MID + wind_90 * MAX_VALUE_SAIL_2

    else:
        wind = 'LEFT'

        if wind_value > 1705:
            percent_wind = (wind_value - 900) / 1060 - 0.75
        else:
            percent_wind = (wind_value - 900) / 1060 + 0.25

        wind_90 = percent_wind - 0.25 - SAIL_OPEN

        if wind_90 == 0:
            do = 'nothing'
        elif wind_90 < 0:
            sail_position = SAIL_MID + wind_90 * MAX_VALUE_SAIL_2
        else:
            sail_position = SAIL_MID - wind_90 * MAX_VALUE_SAIL_2

    msg = "new sail position: %d", sail_position
    log_pub.publish(String(msg))
    rospy.loginfo(msg)
    set_sail(sail_position)


def follow_course_adapted():
    global log_pub
    msg = "follow_course_adapted()"
    log_pub.publish(String(msg))
    rospy.loginfo(msg)

    global sail_position
    global important_values
    global important_values_dif
    global SAIL_MID
    global SAIL_MAX
    global SAIL_MIN
    global SAIL_OPEN
    global WIND_MIN
    global WIND_INTERVAL
    global WIND_MID_BACK
    global WIND_MID_FRONT
    global MAX_VALUE_SAIL_2

    wind_value = important_values[1]
    #1. check wind left or wind right
    msg2 = "new sail position: "
    if wind_value < WIND_MID_BACK and wind_value > WIND_MID_FRONT:
        wind = 'RIGHT'
        msg = "follow_course_adapted()"
        log_pub.publish(String(wind))
        rospy.loginfo(wind)
        wind_percent = (((wind_value - WIND_MIN) / WIND_INTERVAL) - 0.75) * (-2)
        sail_change =  0.5 + SAIL_OPEN - wind_percent
        sail_position = sail_position + sail_change * MAX_VALUE_SAIL_2
        if sail_position>2100:
             sail_position = 2100
        if sail_position<900:
             sail_positon = 900
        log_pub.publish(String(msg2))
        log_pub.publish(String(sail_position))
        rospy.loginfo(msg2)
        rospy.loginfo(sail_position)
        set_sail(sail_position)

    else:
        wind = 'LEFT'
        rospy.loginfo(wind)
        log_pub.publish(String(wind))

        if wind_value > WIND_MID_BACK:
            wind_percent = (((wind_value - WIND_MIN) / WIND_INTERVAL) - 0.75) * (2)
        else:
            wind_percent = (((wind_value - WIND_MIN) / WIND_INTERVAL) + 0.25) * (2)
        sail_change = 0.5 + SAIL_OPEN - wind_percent
        sail_position = sail_position + sail_change * MAX_VALUE_SAIL_2
        if sail_position>2100:
             sail_position=2100
        if sail_position<900:
             sail_position=900
        log_pub.publish(String(msg2))
        log_pub.publish(String(sail_position))
        rospy.loginfo(msg2)
        rospy.loginfo(sail_position)
        set_sail(sail_position)


def start_sailing():
    global log_pub
    msg = "start_sailing()"
    log_pub.publish(String(msg))
    rospy.loginfo(msg)

    global sail_position
    global important_values
    global important_values_dif

        #Berechne Lee
    wind_direction = important_values[1]

    #determine LEFT or RIGHT
        #if LEFT: Sail position has to increase by 10%
        #if RIGHT: Sail position has to decrease by 10%
    if wind_direction > WIND_MIN:
        wind = 'LEFT'
    else:
        wind = 'RIGHT'

    wind_90_degrees = wind_direction + MAX_VALUE_WIND/4
    wind_90_degrees = wind_90_degrees % MAX_VALUE_WIND
    #todo: wind_position übertragen zu Sail_position
    wind_prozent = wind_90_degrees / MAX_VALUE_WIND    #Max-value windsensor
    sail_90_degrees = wind_prozent * MAX_VALUE_SAIL  #Max-value sail-servo



    if sail_90_degrees > SAIL_MAX:
        sail_90_degrees = sail_90_degrees - MAX_VALUE_SAIL/2
    elif sail_90_degrees < SAIL_MIN:
        sail_90_degrees = sail_90_degrees + MAX_VALUE_SAIL

    if wind == 'LEFT':
        sail_position = sail_90_degrees + 0.1 * MAX_VALUE_SAIL    #Max-value sail-servo
    elif wind == 'RIGHT':
        sail_position = sail_90_degrees - 0.1 * MAX_VALUE_SAIL

        #sail_position_pub.publish(sail_position)
    msg2 = "Starting_sail_position: %d", sail_position
    log_pub.publish(String(msg2))
    rospy.loginfo(msg2)

    set_sail(sail_position)


def follow_course():
    global log_pub
    msg = "follow_course()"
    log_pub.publish(String(msg))
    rospy.loginfo(msg)

    global sail_position

        #Berechne die Veränderung der Segelposition aufgrund der Veränderung des Windes!
    wind_dif = important_values_dif[1]
    wind_dif_prozent = wind_dif / MAX_VALUE_WIND     #max-value wind-sensor


    msg2 = "old sail position: %d", sail_position
    log_pub.publish(String(msg2))
    rospy.loginfo(msg2)
        #Rechnung test - vielleicht ein Vorzeichenfehler!
    new_sail_position = sail_position + wind_dif_prozent * MAX_VALUE_SAIL
    msg3 = "new sail position: %d", new_sail_position
    log_pub.publish(String(msg3))
    rospy.loginfo(msg3)
    sail_position = new_sail_position
        # Veröffentliche die Segelposition auf dem Topic "sail_position"
        # if new_sail_position > sail_max: new_sail_position -= sail_max
        #self.sail_position_pub.publish(new_sail_position)
    print(new_sail_position)
    new_sail_position = int(new_sail_position)
    print(new_sail_position)
    set_sail(new_sail_position) 


############################# init - function ################################

def init_sail():
        # Initialisiere Knoten
    global SAIL_MIN
    global SAIL_MID
    global SAIL_MAX

    global WIND_MIN
    global WIND_MID_BACK
    global WIND_MID_FRONT

    print("Starting sailSetter2.py")



        #init_node
    rospy.init_node('sail_setter', anonymous=True)

    global log_pub
    msg = "read calibration data"
    log_pub.publish(String(msg))
    rospy.loginfo(msg)
    #WIND_MIN,WIND_MID,WIND_MAX,SAIL_MIN,SAIL_MID,SAIL_MAX = read_calibration_data()
    #rospy.loginfo(f"Parameters sail: \nMIN: {SAIL_MIN}\nMID: {SAIL_MID}\nMAX: {SAIL_MAX}")
    #rospy.loginfo(f"Parameters wind: \nMIN: {WIND_MIN}\nMID: {WIND_MID}\nMAX: {WIND_MAX}")

    SAIL_MIN = 910
    SAIL_MID = 1500
    SAIL_MAX = 2090

    WIND_MIN = 905
    WIND_MID_BACK = 1705
    WIND_MID_FRONT = 1175
    set_sail(SAIL_MID)
    time.sleep(3)
    set_sail(950)
    set_sail(SAIL_MID)
    
    listen()
