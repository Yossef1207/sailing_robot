import socket
from rudder import *
from sailSetter2 import *
import numpy as np
from colorama import Fore
import rospy

def Diagnostic():
    status_services = np.zeros(11)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM) #roscore check over open port. Port open -> roscore not running
    result = sock.connect_ex(('127.0.0.1',11311)) #check rocore
    if result == 0: #port is open
        self.roscore = False
        status_services[0] = 0
    else:
        self.roscore = True
        status_services[0] = 1
    try:
        self.mavros_subscriber = rospy.Subscriber("mavros")
        status_services[1] = 1
    except:
        self.mavros_subscriber = False
        status_services[1] = 0
    try:
        self.gps_subscriber = rospy.Subscriber("gps")
        status_services[2] = 1
    except:
        self.gps_subscriber = False
        status_services[2] = 0
    try:
        self.yolov7_subscriber = rospy.Subscriber("yolov7")
        status_services[3] = 1
    except:
        self.yolov7_subscriber = False
        status_services[3] = 0
    try:
        self.rudder_subscriber = rospy.Subscriber("rudder")
        working = False
        rudder.steer(1500)
        pos = rudder.get_rudder_postion()
        if pos == 1500:
            working = True
        rudder.steer(2000)
        pos = rudder.get_rudder_postion()
        if pos == 2000:
            working = True
        else:
            working = False
        if working:
            status_services[4] = 1
        else:
            status_services[4] = 2
    except:
        self.rudder_subscriber = False
        status_services[4] = 0
    try:
        self.logging_subscriber = rospy.Subscriber("logging")
        status_services[5] = 1
    except:
        self.logging_subscriber = False
        status_services[5] = 0
    try:
        self.windsensor_subscriber = rospy.Subscriber("windsensor.py")
        status_services[6] = 1
    except:
        self.windsensor_subscriber = False
        status_services[6] = 0
    try:
        self.sailsetter_subscriber = rospy.Subscriber("sailsetter")

        working = False
        sailSetter2.set_sail(1000)
        pos = sailSetter2.get_sail_postion()
        if pos == 1000:
            working = True
        sailSetter2.steer(2000)
        pos = sailSetter2.get_sail_postion()
        if pos == 2000:
            working = True
        else:
            working = False
        if working:
            status_services[7] = 1
        else:
            status_services[7] = 2
    except:
        self.sailsetter_subscriber = False
        status_services[7] = 0
    try:
        self.yoloMediator_subscriber = rospy.Subscriber("yolo_mediator")
        status_services[8] = 1
    except:
        self.yoloMediator_subscriber = False
        status_services[8] = 0
    try:
        self.camera_subscriber = rospy.Subscriber("camera_py")
        status_services[9] = 1
    except:
        self.camera_subscriber = False
        status_services[9] = 0
    try:
        self.imageTransportSubscriber = rospy.Subscriber("image_transport_tutorial")
        status_services[10] = 1
    except:
        self.imageTransportSubscriber = False
        status_services[10] = 0
    
    values = ["RosCore:","Mavros","gps:","yolov7","rudder:","logging:","windsensor:","sailsetter","yoloMediator:","camera:", "imageTransportSubscriber"]
    for i in range(0,10):
        print(values[i], end='')
        if(status_services[i] == 0):
            print(Fore.RED + " not running")
        else:
            print(Fore.GREEN + " running")
            if(((values == 4) or (values == 7)) && status_services == 2):
                print(Fore.RED + "+ is not responding correct")

if __name__ == "__main__":
    Diagnostic()