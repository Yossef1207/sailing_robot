#!/bin/python

import numpy as np
import dronekit as dk
import time
import sys
import json
import math
import threading as th

class BoatThread(th.Thread):
    _stop = False
    _apm = None
    _cnt = 0
    _iter = 0
    _max_iter = 100
    _runtime = 100
    _lock = th.Lock()
    _sail_min = 0
    _sail_mid = 0
    _sail_max = 0
    _rudder_min = 0
    _rudder_mid = 0
    _rudder_max = 0
    _center_wp = (53,13)
    _center_distance = 20
    _wps = ((53.182920,13.147556),
            (53.182855,13.147889),
            (53.182778,13.147309),
            (53.182740,13.147642))

    
    def __init__(self, apm, *args):
        super(BoatThread, self).__init__(*args)
        self._apm = apm


    def updateParams(self, *args):    
        self._lock.acquire()        
        self._sail_min = self._apm.parameters['SAIL_MIN']
        self._sail_mid = self._apm.parameters['SAIL_MID']
        self._sail_max = self._apm.parameters['SAIL_MAX']
        self._wind_min = self._apm.parameters['WIND_MIN']
        self._wind_mid = self._apm.parameters['WIND_MID']
        self._wind_max = self._apm.parameters['WIND_MAX']
        self._lock.release()

        
    def stop(self):
        self._lock.acquire()
        self._stop = True
        self._lock.release()
        print("Stopped: ",self.getName())
        
    def run(self):
#        self.test_wind()
        self.test_value()
        
            
    def getCourseToWaypoint(self, wp, *args):
        self._lock.acquire()
        lat = self._apm.location.global_frame.lat
        lon = self._apm.location.global_frame.lon
        self._lock.release()
        latWP = wp[0]
        lonWP = wp[1]
        dLon = math.radians(lonWP - lon)
        x = math.sin(dLon) * math.cos(latWP)
        y = math.cos(lat) * math.sin(latWP) - (math.sin(lat) * math.cos(latWP) * math.cos(dLon))
        course = (math.degrees(math.atan2(x, y)) + 360) % 360
        return course
    
    
    def getDistanceToWaypoint(self, wp, *args):
        self._lock.acquire()
        lat = self._apm.location.global_frame.lat
        lon = self._apm.location.global_frame.lon
        self._lock.release()
        latWP = wp[0]
        lonWP = wp[1]
        dlat = latWP - lat
        dlon = lonWP - lon
        return math.sqrt((dlat*dlat) + (dlon*dlon)) * 1.113195e5


    def getDistanceToCenter(self, *args):
        return self.getDistanceToWaypoint(self._center_wp)


    def getApparentSail(self, *args):
        self.updateParams()
        self._lock.acquire()
        apparent = self._apm.channels[6]
        self._lock.release()
        return ((((apparent - self._wind_mid) * 3600) / (self._wind_max - self._wind_min)) + 3600) % 3600


    def getApparentBoat(self, *args):
        self.updateParams()
        apparent = self.getApparentSail();
        sail = self.getSail();
        return -((((sail-self._sail_mid)*(60000 / (self._sail_max - self._sail_mid)))/100)-apparent);


    def getTrueWind(self, *args):
        self.updateParams()
        self._lock.acquire()
        heading = self._apm.heading
        self._lock.release()
        apparent = self.getApparentBoat();
        return (heading+apparent) % 3600;
    
        
    def getSail(self, *args):
        self._lock.acquire()
        sail = self._apm.channels[2]
        self._lock.release()
        return sail

    
    def check_course(self, *args):
        sail = getSail()
        apparent = getApparent()
    

    def initialize_apparent(self, target_apparent, target_sail_angle, *args):
        sail = getSail()
        apparent = getApparent()
        if (apparent < 30):
            return True 
        if (apparent > 330):
            return True

        
    def goto_waypoint(self, wp, *args):
        return True

    
    def goto_center(self, *args):
        goto_waypoint(self._center_wp)
        return True


    def go_course(self, course, args):
        self._lock.acquire()
        self._apm.parameters['TARGET_COURSE'] = course
        print("Target Course: %s" % self._apm.parameters['TARGET_COURSE'])
        sys.stdout.flush()
        self._lock.release()
        while True:
            milliseconds = int(round(time.time() * 1000))
            distance = self.getDistanceToCenter()
            if (distance > self._center_distance):
                break;
            milliseconds = (int(round(time.time() * 1000))-milliseconds)
            idle = float(max((self._runtime-milliseconds),0))/1000
            time.sleep(idle)
            self._lock.acquire()
            tmp = self._stop
            self._lock.release()
            if (tmp):
                break
            

    def go_apparent(self, target_apparent, target_sail_angle, args):
        initialize_apparent(target_apparent, target_sail_angle)
        self._lock.acquire()
        self._apm.parameters['TARGET_APPARENT'] = target_apparent
        print("Target Apparent: %s" % self._apm.parameters['TARGET_APPARENT'])
        sys.stdout.flush()
        self._lock.release()
        while True:
            milliseconds = int(round(time.time() * 1000))
            distance = self.getDistanceToCenter()
            if (distance > self._center_distance):
                break;
            milliseconds = (int(round(time.time() * 1000))-milliseconds)
            idle = float(max((self._runtime-milliseconds),0))/1000
            time.sleep(idle)
            self._lock.acquire()
            tmp = self._stop
            self._lock.release()
            if (tmp):
                break

                
    
    def test_waypoints(self, *args):
        nextWP = 0;
        while True:
            milliseconds = int(round(time.time() * 1000))
            distance = self.getDistanceToWaypoint(self._wps[nextWP])
            if (distance < 3):
                nextWP = (nextWP + 1) % 2
            course = self.getCourseToWaypoint(self._wps[nextWP])
            self._lock.acquire()
            self._apm.parameters['TARGET_COURSE'] = course
            print("Target Course: %s" % self._apm.parameters['TARGET_COURSE'])
            sys.stdout.flush()
            self._lock.release()
            milliseconds = (int(round(time.time() * 1000))-milliseconds)
            idle = float(max((self._runtime-milliseconds),0))/1000
            time.sleep(idle)
            self._lock.acquire()
            tmp = self._stop
            self._lock.release()
            if (tmp):
                break


    def test_wind(self, *args):
        while True:
            milliseconds = int(round(time.time() * 1000))
            self.goto_center()
            self.go_course()
            milliseconds = (int(round(time.time() * 1000))-milliseconds)
            idle = float(max((self._runtime-milliseconds),0))/1000
            time.sleep(idle)
            self._lock.acquire()
            tmp = self._stop
            self._lock.release()
            if (tmp):
                break
 

    def test_value(self, *args):
        while True:
            milliseconds = int(round(time.time() * 1000))
            print("ApparentSail: %s" % self.getApparentSail())            
            print("ApparentBoat: %s" % self.getApparentBoat())            
            print("TrueWind: %s" % self.getTrueWind())            
            print("Channels: %s" % self._apm.channels)            
            milliseconds = (int(round(time.time() * 1000))-milliseconds)
            idle = float(max((self._runtime-milliseconds),0))/1000
            time.sleep(idle)
            self._lock.acquire()
            tmp = self._stop
            self._lock.release()
            if (tmp):
                break
                

                
if __name__ == '__main__':
    apm = dk.connect('/dev/ttyACM0', wait_ready=True, baud=115200)
    bt = BoatThread(apm)
    bt.start()
    time.sleep(12)
    bt.stop()
    bt.join()

