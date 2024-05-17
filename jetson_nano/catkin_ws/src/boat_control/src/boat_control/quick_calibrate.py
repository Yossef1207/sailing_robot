import json
import os
import rospy
import time
from std_msgs.msg import String

from boat_control import apm

#Calibration json file can be obtained by running the "calibration" script from the "boat_control" package
CALIB_FILE = "calibration_data.json"
CONFIG_FILE = "config.json"
retry_delay = 10

log_topic = "/logmsg"

def set_values_apm(data):
    apm.set_apm_mode("MANUAL")
    for key, value in data.items():
        apm.set_parameter(key, value)

def values_apm_set(data):
    try:
        for key, value in data.items():
            if not (apm.get_parameter(key) == value):
                return False
        return True
    except:
        return False

def get_data(filename):
    __location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))
    file_path = os.path.join(__location__, filename)
    with open(file_path, 'r') as f:
        data = json.loads(f.read())
    return data

def quick_calibration():
    print("calibrating...")

    global CALIB_FILE
    global CONFIG_FILE
    calib_data = get_data(CALIB_FILE)
    config_data = get_data(CONFIG_FILE)
    data = {**calib_data, **config_data}
    if data["R_SERVO_UPRIGHT"] != 0:
        min_tmp = data["RUDDER_MIN"]
        data["RUDDER_MIN"] = data["RUDDER_MAX"]
        data["RUDDER_MAX"] = min_tmp

    rospy.init_node("apm_quick_calibrate", anonymous=True)

    global log_topic
    global retry_delay
    log_pub = rospy.Publisher(log_topic, String, queue_size=1)
    try:
        set_values_apm(data)
    except Exception as e:
        print(e)
    if not values_apm_set(data):
        print(f"Setting Calibration failed, retrying in {retry_delay} seconds")
        time.sleep(retry_delay)
        try:
            set_values_apm(data)
        except Exception as e:
            print(e)
        finally:
            if not values_apm_set(data):
                print("Setting Calibration failed, exiting")
                log_pub.publish(String("Quick calibration failed"))
                exit()

    print(data)
    log_pub.publish(String(data))

    print("quick calibration finished")


if __name__ == "__main__":
    quick_calibration()
