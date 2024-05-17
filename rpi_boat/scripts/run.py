#!/bin/python

import numpy as np
import dronekit as dk
import multiprocessing as mp
import boat
import time

cfg = np.loadtxt('../config/boat.cfg')
# r_left, r_mid, r_right, s_left, s_mid, s_right, wind_zero, wind_min, wind_max

lock = mp.Lock()
apm = dk.connect('/dev/ttyACM0', wait_ready=True, baud=115200)

apm.parameters['RUDDER_MIN'] = cfg[0]
apm.parameters['RUDDER_MID'] = cfg[1]
apm.parameters['RUDDER_MAX'] = cfg[2]
apm.parameters['SAIL_MIN'] = cfg[3]
apm.parameters['SAIL_MID'] = cfg[4]
apm.parameters['SAIL_MAX'] = cfg[5]
apm.parameters['WIND_MID'] = cfg[6]
apm.parameters['WIND_MIN'] = cfg[7]
apm.parameters['WIND_MAX'] = cfg[8]

print("RUDDER_MID",apm.parameters['RUDDER_MID'])
print("SAIL_MID",apm.parameters['SAIL_MID'])
print("WIND_MID",apm.parameters['WIND_MID'])

apm.mode = 'MANUAL'
print(apm.mode)

lt = boat.LogThread(apm)
cp = boat.CameraProcess(lock)
bt = boat.BoatThread(apm)

apm.parameters['SAIL_STEP'] = 10
apm.parameters['SAIL_GAP'] = 40
apm.parameters['TARGET_COURSE'] = 0
apm.parameters['TARGET_MODE'] = 2

lt.start()
cp.start()
#bt.start()

def stop_all():
    apm.parameters['TARGET_MODE'] = 0
    lt.stop()
    cp.stop()
#    bt.stop()

def tm(mode):
    apm.parameters['TARGET_MODE'] = mode

# port tack, i.e., wind from port    
def pt(angle,apparent):
    apm.parameters['SAIL_ANGLE'] = cfg[4] + angle
    apm.parameters['TARGET_APPARENT'] = -apparent

# starboard tack, i.e., wind from starboard    
def st(angle,apparent):
    apm.parameters['SAIL_ANGLE'] = cfg[4] - angle
    apm.parameters['TARGET_APPARENT'] = apparent

# tack to port tack, i.e., wind from port    
def t2pt(angle,apparent,delay=1):
    apm.parameters['SAIL_ANGLE'] = cfg[4] - 200
    apm.parameters['TARGET_APPARENT'] = 200    
    time.sleep(delay)
    apm.parameters['TARGET_APPARENT'] = -300        
    time.sleep(delay)    
    apm.parameters['SAIL_ANGLE'] = cfg[4] + angle
    apm.parameters['TARGET_APPARENT'] = -apparent

# tack to starboard tack, i.e., wind from starboard    
def t2st(angle,apparent,delay=1):
    apm.parameters['SAIL_ANGLE'] = cfg[4] + 200
    apm.parameters['TARGET_APPARENT'] = -200    
    time.sleep(delay)
    apm.parameters['TARGET_APPARENT'] = 300        
    time.sleep(delay)    
    apm.parameters['SAIL_ANGLE'] = cfg[4] - angle
    apm.parameters['TARGET_APPARENT'] = apparent
    
    
# use python -i run.py or uncomment following lines
#lt.join()
#cp.join()
#bt.join()
