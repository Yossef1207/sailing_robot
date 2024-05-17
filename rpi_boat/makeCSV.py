#!/usr/bin/python

import os
import sys
import glob
import numpy
import glob
import json
import copy
import csv

logs = []
cfg = []
dr = { 'wind_min'  : 911,
       'wind_mid'  : 1170,
       'wind_max'  : 1952,
       'sail_min'  : 600,
       'sail_mid'  : 1300,
       'sail_max'  : 2000,
       'rudder_min' : 700,
       'rudder_mid' : 1300,
       'rudder_max' : 1900,
       'idx'       : 0,       
       'timestamp' : 0,
       'voltage'   : 8.4,
       'current'   : 0,
       'battery'   : 0,
       'yaw'       : 0,
       'pitch'     : 0,
       'roll'      : 0,
       'servo1'    : 0,
       'servo2'    : 0,
       'servo3'    : 0,
       'servo4'    : 0,
       'windspeed' : 0,
       'winddir'   : 0,
       'rudder'    : 0,
       'sail'      : 0,
       'velocity.x': 0,       
       'velocity.y': 0,       
       'velocity.z': 0,       
       'sog'       : 0,
       'heading'   : 0,
       'lat'       : 0,
       'lon'       : 0,
       'alt'       : 0,
       'apparent_s': 0,
       'apparent_b': 0,       
       'true_calc' : 0}      

def getApparentSail(apparent):
    return ((((apparent - wind_mid) * 3600) / (wind_max - wind_min)) + 3600) % 3600

def getApparentBoat(apparent,sail):
    app = getApparentSail(apparent)
    return -((((sail-sail_mid)*(65000 / (sail_max - sail_mid)))/100)-apparent);

def getTrueWind(apparent,sail,heading):
    apparent = getApparentBoat(apparent,sail)
    return (heading+apparent) % 3600

def setApparentSail(dr):
    dr['apparent_s'] = ((((dr['winddir'] - dr['wind_mid']) * 3600) / (dr['wind_max'] - dr['wind_min'])) + 3600) % 3600

def setApparentBoat(dr):
    sail = (((dr['sail']-dr['sail_mid'])*(65000 / (dr['sail_max'] - dr['sail_mid'])))/100)
    dr['apparent_b'] = -(sail - dr['apparent_s'])

def setTrueWind(dr):
    dr['true_calc'] = (dr['heading'] + dr['apparent_b']) % 3600

def readLogs(str='/media/sf_D_DRIVE/data/temp/tmpMTECSail/jsons/210930/*'):
    for fn in sorted(glob.glob(str)):
        log = []
        print(fn)
        for line in open(fn,'r'):
            try:
                d = json.loads(line)
                logs.append(d)
            except:
                print(line)

def readCfg(str='../config/boat.cfg'):
    cfg = np.loadtxt(str)
    dr['wind_min'] = cfg[7]
    dr['wind_mid'] = cfg[6]
    dr['wind_max'] = cfg[8]
    dr['sail_min'] = cfg[3]
    dr['sail_mid'] = cfg[4]
    dr['sail_max'] = cfg[5]
    dr['rudder_min'] = cfg[0]
    dr['rudder_mid'] = cfg[1]
    dr['rudder_max'] = cfg[2]

def convertLogs(logs):
    tx  = []
    idx = 0
    for ds in logs:
       dr['idx'] = idx        
       dr['timestamp'] = ds[0]
       dr['voltage'] = ds[1]
       dr['current'] = ds[2]
       dr['battery'] = ds[3]
       dr['yaw'] = ds[4]
       dr['pitch'] = ds[5]
       dr['roll'] = ds[6]
       dr['servo1'] = ds[7][u'1']
       dr['servo2'] = ds[7][u'2']
       dr['servo3'] = ds[7][u'3']
       dr['servo4'] = ds[7][u'4']
       dr['windspeed'] = ds[7][u'5']
       dr['winddir'] = ds[7][u'6']
       dr['rudder'] = ds[9]
       dr['sail'] = ds[10]
       dr['velocity.x'] = ds[11][0]
       dr['velocity.y'] = ds[11][1]
       dr['velocity.z'] = ds[11][2]       
       dr['sog'] = ds[12]        
       dr['heading'] = ds[13]
       dr['lat'] = ds[14]
       dr['lon'] = ds[15]
       dr['alt'] = ds[16]
       setApparentSail(dr)
       setApparentBoat(dr)
       setTrueWind(dr)
       tx.append(copy.deepcopy(dr))
       idx = idx + 1
    return tx

def writeCSV(tx, str='/media/sf_D_DRIVE/data/temp/tmpMTECSail/jsons/210930/jsons_fmt_x.csv'):
    try:
        with open(str, 'w') as f:
            writer = csv.DictWriter(f, fieldnames=list(dr.keys()))
            writer.writeheader()
            for elem in tx:
                writer.writerow(elem)
    except IOError:
        print("I/O error")

#    numpy.savetxt(str,tx,delimiter=",",fmt='%6.8f')



#       dr['apparent_s'] = getApparentSail(ds[7][u'6'])
#       dr['apparent_b'] = getApparentBoat(ds[7][u'3'],ds[7][u'6'])
#       dr['true_calc'] = getTrueWind(ds[7][u'3'],ds[7][u'6'],ds[11])])


