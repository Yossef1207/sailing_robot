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
            log_record = [milliseconds,
                          self._apm.battery.voltage,
                          self._apm.battery.current,
                          self._apm.battery.level,
                          self._apm.attitude.yaw,
                          self._apm.attitude.pitch,
                          self._apm.attitude.roll,
                          self._apm.channels,
                          self._apm.channels.overrides,
                          self._apm.groundspeed,
                          self._apm.heading,
                          self._apm.location.global_frame.lat,
                          self._apm.location.global_frame.lon,
                          self._apm.location.global_frame.alt]
            f.write(json.dumps(log_record))
            f.write('\n')
            milliseconds = (int(round(time.time() * 1000))-milliseconds)
            idle = float(max((self._runtime-milliseconds),0))/1000
            time.sleep(idle)
            with self._stop.get_lock():                        
                if (self._stop.value):
                    break;
        f.flush()
        f.close()
                
    
if __name__ == '__main__':
#    mp.set_start_method('spawn')
    lock = mp.Lock()
    apm = dk.connect('/dev/ttyACM0', wait_ready=True, baud=115200)
    lp = LogProcess(lock,apm)
    lp.start()
#    time.sleep(12)
#    lp.stop()
    lp.join()

