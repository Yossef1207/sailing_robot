#!/bin/python

import sys
import tty
import termios
import numpy as np
from dronekit import connect

EOT = '\x04'  # CTRL+D
ESC = '\x1b'
CSI = '['
RET = '\r'
line = ''


def getchar():
    fd = sys.stdin.fileno()
    attr = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, attr)


def rudder(v):
    vehicle.channels.overrides[1] = v
    
def sail(v):
    vehicle.channels.overrides[3] = v

    
def calib(start, f):
    tmp = start
    print(tmp)
    while True:
        c = getchar()
        if c == EOT:
            print('exit')
            break
        elif c == RET:
            return tmp
            break      
        elif c == '\x03':
            print('exit')
            break
        elif c == ESC:
            if getchar() == CSI:
                x = getchar()
                if x == 'A':
                    tmp = tmp + 10
                    f(tmp)                    
                    print(tmp)                    
#                    print('UP')
                elif x == 'B':
                    tmp = tmp - 10
                    f(tmp)                    
                    print(tmp)                                        
#                    print('DOWN')
                elif x == 'C':
                    tmp = tmp + 1
                    f(tmp)
                    print(tmp)
#                    print('RIGHT')
                elif x == 'D':
                    tmp = tmp - 1
                    f(tmp)                    
                    print(tmp)
#                    print('LEFT')

def calib_wind():
    min_v = vehicle.channels[6]
    max_v = vehicle.channels[6]
    while True:
        tmp = vehicle.channels[6]
        print tmp
        min_v = min(min_v, tmp)
        max_v = max(max_v, tmp)        
        c = getchar()
        if c == EOT:
            print('exit')
            break
        elif c == RET:
            return tmp, min_v, max_v
            break      


print("Servo calibration, exit with CTRL+C")
vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=115200, use_native=False)
vehicle.mode = "MANUAL"
vehicle.parameters['TARGET_MODE'] = 3

vehicle.parameters['RUDDER_MIN'] = 1500
vehicle.parameters['RUDDER_MID'] = 1500
vehicle.parameters['RUDDER_MAX'] = 1500
vehicle.parameters['SAIL_MIN'] = 1500
vehicle.parameters['SAIL_MID'] = 1500
vehicle.parameters['SAIL_MAX'] = 1500
vehicle.parameters['WIND_MID'] = 1500
vehicle.parameters['WIND_MIN'] = 1500
vehicle.parameters['WIND_MAX'] = 1500

print("Rudder left calibration")
r_left = calib(1300,rudder)
print("Rudder right calibration")
r_right = calib(1300,rudder)
print("Rudder midships calibration")
r_mid = calib(1300,rudder)

print("Sail left calibration")
s_left = calib(1300,sail)
print("Sail right calibration")
s_right = calib(1300,sail)
print("Sail midships calibration")
s_mid = calib(1300,sail)

print("Wind vane calibration: rotate a few time while pressing <space>, then align from front and press enter")
wind_zero, wind_min, wind_max = calib_wind()

np.savetxt('../config/boat.cfg', [r_left, r_mid, r_right, s_left, s_mid, s_right, wind_zero, wind_min, wind_max])

