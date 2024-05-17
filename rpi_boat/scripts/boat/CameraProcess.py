#!/bin/python

import numpy as np
import cv2
import time
import multiprocessing as mp

class CameraProcess(mp.Process):
    _stop = mp.Value('b', False)
    _width = 2560
    _hight = 720
    _fps = 0    
    _mid = _width // 2
    _video = None
    _cnt = 0
    _runtime = 200
    
    def __init__(self, lock, *args):
        super(CameraProcess, self).__init__(*args)
        self._lock = lock
        self._video = cv2.VideoCapture(0)
        self._video.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._video.set(cv2.CAP_PROP_FRAME_HEIGHT, self._hight)
        self._video.set(cv2.CAP_PROP_FPS, self._fps)

    def stop(self):
        with self._stop.get_lock():
            self._stop.value = True
        print("Stopped: ",self.name)            

    def setRuntime(self, runtime):
        with self._runtime.get_lock():
            self._runtime.value = runtime

    def run(self):
        start_time = str(int(round(time.time() * 1000)))        
        while True:            
            milliseconds = int(round(time.time() * 1000))
            ret, frame = self._video.read()
            if (ret):
                cv2.imwrite('../log/CamLog_'+str(start_time)+'_'+str(self._cnt)+'_'+str(milliseconds)+'.jpg',frame)
                self._cnt += 1
            milliseconds = (int(round(time.time() * 1000))-milliseconds)
            idle = float(max((self._runtime-milliseconds),0))/1000
#            print(self._runtime-milliseconds)
            time.sleep(idle)
            with self._stop.get_lock():                        
                if (self._stop.value):
                    break;
    
if __name__ == '__main__':
#    mp.set_start_method('spawn')
    lock = mp.Lock()
    cp = CameraProcess(lock)
    cp.start()
#    time.sleep(12)
#    cp.stop()
    cp.join()

