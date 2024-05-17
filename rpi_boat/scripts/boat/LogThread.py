#!/bin/python

import numpy as np
import dronekit as dk
import time
import json
import threading as th

class LogThread(th.Thread):
    _stop = False
    _apm = None
    _cnt = 0
    _iter = 0
    _max_iter = 100
    _runtime = 100
    _lock = th.Lock()
    _sail = 1500
    _rudder = 1500
    
    def __init__(self, apm, *args):
        super(LogThread, self).__init__(*args)
        self._apm = apm
        @apm.on_message('SERVO_OUTPUT_RAW')
        def listener(self, name, msg):
            LogThread._lock.acquire()
            LogThread._rudder = msg.servo1_raw            
            LogThread._sail = msg.servo3_raw
            LogThread._lock.release()
                
    def stop(self):
        self._lock.acquire()
        self._stop = True        
        self._lock.release()
        print("Stopped: ",self.getName())

    def servoListener(self, name, msg):
        LogThread._lock.acquire()
        LogThread._rudder = msg.servo1_raw            
        LogThread._sail = msg.servo3_raw
        LogThread._lock.release()
                
    def run(self):        
        
        start_time = str(int(round(time.time() * 1000)))
        f = open('../log/Log_'+start_time+'_'+str(self._cnt)+'.json','a')
        while True:
            milliseconds = int(round(time.time() * 1000))
            self._iter += 1
            if (self._iter == self._max_iter):
                self._cnt += 1
                self._iter = 0
                f.flush()
                f.close()
                f = open('../log/Log_'+start_time+'_'+str(self._cnt)+'.json','a')
            self._lock.acquire()
            log_record = [milliseconds,
                          self._apm.battery.voltage,
                          self._apm.battery.current,
                          self._apm.battery.level,
                          self._apm.attitude.yaw,
                          self._apm.attitude.pitch,
                          self._apm.attitude.roll,
                          self._apm.channels,
                          self._apm.channels.overrides,
                          LogThread._rudder,
                          LogThread._sail,
                          self._apm.velocity,                          
                          self._apm.groundspeed,
                          self._apm.heading,
                          self._apm.location.global_frame.lat,
                          self._apm.location.global_frame.lon,
                          self._apm.location.global_frame.alt]
            self._lock.release()
            f.write(json.dumps(log_record))
            f.write('\n')
            milliseconds = (int(round(time.time() * 1000))-milliseconds)
            idle = float(max((self._runtime-milliseconds),0))/1000
            time.sleep(idle)
            self._lock.acquire()
            tmp = self._stop
            self._lock.release()
            if (tmp):
                break;
        f.flush()
        f.close()
                
    
if __name__ == '__main__':
    apm = dk.connect('/dev/ttyACM0', wait_ready=True, baud=115200)
    lt = LogThread(apm)
    lt.start()
    time.sleep(12)
    lt.stop()
    lt.join()

