import os
import numpy as np
import time
import json
import multiprocessing as mp
import threading as th
import ros_numpy

import dronekit as dk
import rospy
import std_msgs.msg
import sensor_msgs
import cv2

class LogAPMProcess(mp.Process):
    _stop = mp.Value('b', False)
    _apm = None
    _cnt = 0
    _iter = 0
    _max_iter = 100
    _runtime = 2000
    _thread_lock = th.Lock()
    _rudder = 0
    _sail = 0

    def __init__(self, lock, apm, logging_dir, *args):
        super(LogAPMProcess, self).__init__(*args)
        self._lock = lock
        self._apm = apm
        self._logging_dir = logging_dir
        @apm.on_message('SERVO_OUTPUT_RAW')
        def listener(self, name, msg):
            with LogAPMProcess._thread_lock:
                LogAPMProcess._rudder = msg.servo1_raw
                LogAPMProcess._sail = msg.servo3_raw

    def stop(self):
        with self._stop.get_lock():
            self._stop.value = True

    def run(self):
        start_time = str(int(round(time.time() * 1000)))
        try:
            f = open(self._logging_dir + '/Log_' + start_time + '_' + str(self._cnt) + '.json', 'a')
        except:
            time.sleep(5)
            f = open(self._logging_dir + '/Log_' + start_time + '_' + str(self._cnt) + '.json', 'a')
        while True:
            try:
                milliseconds = int(round(time.time() * 1000))
                self._iter += 1
                if (self._iter == self._max_iter):
                    self._cnt += 1
                    self._iter = 0
                    try:
                        f.flush()
                        f.close()
                    except:
                        time.sleep(1)
                        f.flush()
                        f.close()
                    try:
                        f = open(self._logging_dir + '/Log_' + start_time + '_' + str(self._cnt) + '.json', 'a')
                    except:
                        time.sleep(1)
                        f = open(self._logging_dir + '/Log_' + start_time + '_' + str(self._cnt) + '.json', 'a')
                    log_record = [{"ms": milliseconds},\
                                  {"bat_vol": self._apm.battery.voltage},\
                                  {"bat_cur": self._apm.battery.current},\
                                  {"bat_lev": self._apm.battery.level},\
                                  {"att_yaw": self._apm.attitude.yaw},\
                                  {"att_pit": self._apm.attitude.pitch},\
                                  {"att_roll": self._apm.attitude.roll},\
                                  {"apm_channels": self._apm.channels},\
                                  {"apm_channels_overrides": self._apm.channels.overrides},\
                                  {"apm_grspeed": self._apm.groundspeed},\
                                  {"apm_heading": self._apm.heading},\
                                  {"apm_loc_lat": self._apm.location.global_frame.lat},\
                                  {"apm_loc_lon": self._apm.location.global_frame.lon},\
                                  {"apm_loc_alt": self._apm.location.global_frame.alt},\
                                  {"rudder_raw": LogAPMProcess._sail},\
                                  {"sail_raw": LogAPMProcess._rudder}]
                    f.write(json.dumps(log_record))
                    f.write('\n')
                    print("Log APM written")
                    milliseconds = (int(round(time.time() * 1000)) - milliseconds)
                    idle = float(max((self._runtime - milliseconds), 0)) / 1000
                    time.sleep(idle)
                    with self._stop.get_lock():
                        if (self._stop.value):
                            break
            except:
                try:
                    f.flush()
                    f.close()
                except:
                    pass
                break
        try:
            f.flush()
            f.close()
        except:
            time.sleep(1)
            f.flush()
            f.close()



class LogROSProcess(mp.Process):
    _stop = mp.Value('b', False)

    def __init__(self, lock, logging_dir, *args):
        super(LogROSProcess, self).__init__(*args)
        self._lock = lock
        self._logging_dir = logging_dir

    def stop(self):
        with self._stop.get_lock():
            self._stop.value = True

    def write_log(self, msg):
        try:
            milliseconds = int(round(time.time() * 1000))

            #sender = msg._connection_header
            log_record = [{"ms": milliseconds},
                          {"msg": msg.data}]

            with open(self._logging_dir + '/Log_ROS_' + self._start_time + '.json', 'a') as f:
                f.write(json.dumps(log_record))
                f.write('\n')
            print("Log ROS written")
        except Exception as e:
            print(e)

    def log_image(self, msg):
        print("logging Image")
        try:
            frame = ros_numpy.numpify(msg)
            timestamp = time.time()
            path = f"{self._logging_dir}/Log_CAM_{timestamp}.jpeg"
            cv2.imwrite(path, frame)
            print("Camera Image written")
        except Exception as e:
            print(e)

    def run(self):
        self._start_time = str(int(round(time.time() * 1000)))
        rospy.init_node("LoggingNode")
        self.subscriber = rospy.Subscriber("/logmsg", std_msgs.msg.String, self.write_log)
        rospy.Subscriber('/camera/frame_both', sensor_msgs.msg.Image, self.log_image)
        rospy.spin()





def init_logging():
    lock = mp.Lock()
    try:
        apm = dk.connect('/dev/ttyACM0', wait_ready=True, baud=115200)
    except Exception as e:
        print(e)
        print("Retrying in 5 Seconds")
        time.sleep(5)
        try:
            apm = dk.connect('/dev/ttyACM0', wait_ready=True, baud=115200)
        except:
            print("Dronekit connection failed")

    logging_dir = "/home/nano/isp-2022/jetson_nano/catkin_ws/src/loggingNode/log"
    try:
        os.makedirs(logging_dir, exist_ok = True)
    except OSError as error:
        print(f"Logging directory \"{logging_dir}\" can not be created: {error}")
        print("Retrying in 5 Seconds")
        time.sleep(5)
        try:
            os.makedirs(logging_dir, exist_ok = True)
        except:
            print("Creating logging directory failed")

    apm_log_process = LogAPMProcess(lock, apm, logging_dir)
    ros_log_process = LogROSProcess(lock, logging_dir)
    
    apm_log_process.start()
    ros_log_process.start()

    apm_log_process.join()
    ros_log_process.join()

if __name__ == '__main__':
    init_logging()
