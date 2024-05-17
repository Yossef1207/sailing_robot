#!/bin/python

import numpy as np
import dronekit as dk
import time
import json
import multiprocessing as mp

class LogProcess(mp.Process):
    _stop = mp.Value('b', False)
    _apm = None
    _cnt = 0
    _iter = 0
    _max_iter = 100    
    _runtime = 100
    
    def __init__(self, lock, apm, *args):
        super(LogProcess, self).__init__(*args)
        self._lock = lock
        self._apm = apm        

    def stop(self):
        with self._stop.get_lock():
            self._stop.value = True
            
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
            with self._stop.get_lock():                        
                if (self._stop.value):
                    break;
                
    
if __name__ == '__main__':
    lock = mp.Lock()
    apm = dk.connect('/dev/ttyACM0', wait_ready=True, baud=115200)
    print(apm)
    for i in range(1,50):
        milliseconds = int(round(time.time() * 1000))
        log_record = [milliseconds,apm.battery.voltage,apm.channels]
        print(apm)        
        print(log_record)
        time.sleep(0.1)
        
    lp = LogProcess(lock,apm)
    lp.start()
    time.sleep(5)
    lp.stop()
    lp.join()
    
    for i in range(1,50):
        milliseconds = int(round(time.time() * 1000))
        log_record = [milliseconds,apm.battery.voltage,apm.channels]
        print(apm)        
        print(log_record)
        time.sleep(0.1)
