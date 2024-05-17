#!/usr/bin/env python2.7

# Provide a website, where a route containing multiple targets can be selected and started.

import rospy

from boat_control import apm
import math

from mavros_msgs.msg import GlobalPositionTarget
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from flask import Flask, request, render_template
app = Flask(__name__)

currentPosition = None # Current position (Only needed for return target)
targets = [] # Targets for the boat
currentTargetIndex = -1 # Index of current target
startRouteNow = False # Whether to start the route

gps_target_topic = "/mavros/setpoint_raw/target_global"
gps_position_topic = "/mavros/global_position/global"
log_topic = "/logmsg"

print(
    "Make sure that MAVROS master node is running, e.g. roslaunch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 fcu_protocol:=v1.0"
)

# Update (new) target based on new position
def update_position(position:NavSatFix):
    global targets
    global currentTargetIndex
    global startRouteNow
    global currentPosition
    currentPosition = position
    if(startRouteNow): # Targets available
        distanceInMeters = distanceCoordinatesToMeters(position,targets[currentTargetIndex])
        gui_log(f"Distance to target in meters: {distanceInMeters}")
        if(distanceInMeters <= 10): # Close enough to current target. Provide next target
            currentTargetIndex += 1
            if(currentTargetIndex == len(targets)): # Final target reached
                return finishedRoute()
            # Publish next target
            gui_target_publisher.publish(targets[currentTargetIndex])
            gui_log(f"Published new target: {targets[currentTargetIndex]}")

# Calculate distance from current position to current target
def distanceCoordinatesToMeters(point1,point2):
    p = math.pi/180
    lat1,lat2 = float(point1.latitude), float(point2.latitude)
    lon1,lon2 = float(point1.longitude), float(point2.longitude)
    gui_log(f"{lat1}   {lat2}    {lon1}   {lon2}")
    a = 0.5 - math.cos((lat2-lat1)*p)/2 + math.cos(lat1*p) * math.cos(lat2*p) * (1-math.cos((lon2-lon1)*p))/2
    return 12742 * 1000 * math.asin(math.sqrt(a))

# Log function
def gui_log(message):
    print(message)
    rospy.loginfo(message)
    gui_log_publisher.publish(String(message))

# Init ros node and add subscribers/publishers
rospy.init_node("gui", anonymous=True)
gui_position_subscriber = rospy.Subscriber(
    gps_position_topic, NavSatFix, update_position)
gui_target_publisher = rospy.Publisher(
    gps_target_topic, GlobalPositionTarget, queue_size=1)
gui_log_publisher = rospy.Publisher(
    log_topic, String, queue_size=1)
gui_log("GUI initialization complete.")

# default page
@app.route('/')
def login():
    return render_template('./gui.html', targets=targets)

# Add target coordinates to route
@app.route('/add_target',methods = ['POST', 'GET'])
def add_target():
    if request.method == 'POST':
        global targets
        global currentTargetIndex
        global startRouteNow
        if(not startRouteNow):
            newTarget = GlobalPositionTarget()
            newTarget.longitude = float(request.form.get('lo'))
            newTarget.latitude = float(request.form.get('la'))
            gui_log(f"Add target: {newTarget}")
            targets.append(newTarget)
            currentTargetIndex = 0
        return render_template('./gui.html', targets=targets)

# Start route
@app.route('/start_route',methods = ['POST', 'GET'])
def start_route():
    if request.method == 'POST':
        global targets
        global currentTargetIndex
        global startRouteNow
        global currentPosition
        if(currentTargetIndex != 0): # No targets in route
            return finishedRoute()
        # Add current position as return target
        if(currentPosition != None and\
           currentPosition.longitude != 0.0 and currentPosition.latitude != 0.0):
            newTarget = GlobalPositionTarget()
            newTarget.longitude = float(currentPosition.longitude)
            newTarget.latitude = float(currentPosition.latitude)
            gui_log(f"Add return target: {newTarget}")
            targets.append(newTarget)
        gui_log(f"Starting route with {len(targets)} target(s).")
        gui_target_publisher.publish(targets[currentTargetIndex])
        startRouteNow = True
        return render_template('./gui.html', targets=targets)

@app.route('/reset_route',methods = ['POST', 'GET'])
def reset_route():
    if request.method == 'POST':
        global targets
        gui_log(f"Reset route with {len(targets)} target(s).")
        reset()
        return render_template('./gui.html', targets=targets)

# Final target of route reached
def finishedRoute():
    global currentTargetIndex
    gui_log(f"Route finished. Reached {currentTargetIndex} target(s).")
    reset()

# Reset variables
def reset():
    global startRouteNow
    global currentTargetIndex
    global targets
    startRouteNow = False
    currentTargetIndex = -1
    targets = []

# Initialize gui node and flask app
app.run(host='192.168.0.112',port=5000,debug = True)

def init_gui():
  rospy.spin()
