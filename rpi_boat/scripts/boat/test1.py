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
    
    def __init__(self, apm, *args):
        super(LogThread, self).__init__(*args)
        self._apm = apm        

    def stop(self):
        self._stop = True
            
    def run(self):
        start_time = str(int(round(time.time() * 1000)))
        while True:
            milliseconds = int(round(time.time() * 1000))
            log_record = [milliseconds,self._apm.battery.voltage,self._apm.channels]
            print(apm)
            print(self._apm)                        
            print(log_record)
            print([milliseconds,apm.battery.voltage,apm.channels])
            milliseconds = (int(round(time.time() * 1000))-milliseconds)
            idle = float(max((self._runtime-milliseconds),0))/1000
            time.sleep(idle)
            if (self._stop):
                break;
                
    
if __name__ == '__main__':
    apm = dk.connect('/dev/ttyACM0', wait_ready=True, baud=115200)
    print(apm)
    for i in range(1,50):
        milliseconds = int(round(time.time() * 1000))
        log_record = [milliseconds,apm.battery.voltage,apm.channels]
        print(apm)        
        print(log_record)
        time.sleep(0.1)
        
    lp = LogThread(apm)
    lp.start()
    time.sleep(5)
    lp.stop()
    
    for i in range(1,50):
        milliseconds = int(round(time.time() * 1000))
        log_record = [milliseconds,apm.battery.voltage,apm.channels]
        print(apm)        
        print(log_record)
        time.sleep(0.1)
