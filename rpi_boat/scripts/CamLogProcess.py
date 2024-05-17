#!/bin/python

import numpy as np
import cv2
from dronekit import connect
import time
import multiprocessing as mp

class CamLogProcess(mp.Process):
    _lock = mp.Lock()
    _stop = mp.Value('b', False)
    _width = 2560
    _hight = 720
    _mid = _width // 2
    _video = None
    _cnt = 0
    _left_img = None
    _right_img = None    
    
    def __init__(self, lock, *args):
        super(CamLogProcess, self).__init__(*args)
        self._lock = lock
        self._video = cv2.VideoCapture(0)
        self._video.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        self._video.set(cv2.CAP_PROP_FRAME_HEIGHT, self._hight)        

    def stop(self):
#        with self._stop.get_lock():
#            self._stop = True
#        print('stopped',self._stop)
        print(11,self._lock)                                        
        self._lock.acquire()
        self._stop = True
        print(12,self._lock)                                                
        self._lock.release()
        print(13,self._lock)                                                
        print('stopped',self._stop)        

    def run(self):
        while True:
            ret, frame = self._video.read()
            print(1,self._lock)
            if (ret):
                cv2.imwrite('log/CamLog_'+str(int(round(time.time() * 1000)))+'_'+str(self._cnt)+'.png',frame)
                print(2,self._lock)                
                self._cnt += 1
                print(3,self._lock)                                
                self._lock.acquire()
                print(4,self._lock)                                
                self._left_img = frame[:, :self._mid]
                self._right_img = frame[:, self._mid:]
                print(5,self._lock)                                
                self._lock.release()
                print(6,self._lock)                                
                time.sleep(0.05)
                print(7,self._lock)                                                
            print(ret, self._stop)
            print(8,self._lock)                                                            
            self._lock.acquire()
            print(9,self._lock)                                                            
            tmp = self._stop
            self._lock.release()
            print(10,self._lock,tmp)                                                            
            if (tmp):
                break;

    def getLeft(self):
        self._lock.acquire()
        tmp = np.copy(self._left_img)
        self._lock.release()        
        return tmp

    def getRight(self):
        self._lock.acquire()
        tmp = np.copy(self._right_img)
        self._lock.release()        
        return tmp
    
if __name__ == '__main__':
#    mp.set_start_method('spawn')
    lock = mp.Lock()
    clp = CamLogProcess(lock)
    clp.start()
    time.sleep(12)
    clp.stop()
    clp.join()

