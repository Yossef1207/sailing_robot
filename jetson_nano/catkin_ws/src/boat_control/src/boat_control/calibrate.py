from typing import Tuple, Dict
from enum import Enum
import sys
import tty
import termios
import json

import rospy
import os

from boat_control import apm
 

class ServoType(Enum):
    RUDDER = 1
    SAIL = 3

    @property
    def rc_channel(self) -> int:
        return self.value


WIND_SENSOR_RC_CHANNEL = 6

# Characters to handle in servo calibration loops
EOT = "\x04"  # End Of Transmission (CTRL-D)
ETX = "\x03"  # End Of Text (CTRL-C)
ESC = "\x1b"  # ASCII Escape character (^[)
CSI = "["  # Control Sequence Introducer ([)
RET = "\r"  # Carriage return (ENTER)
UP = "A"
DOWN = "B"
RIGHT = "C"
LEFT = "D"


def calibrate():
    print("Servo calibration, exit with CTRL+C")
    print(
        "Make sure that MAVROS master node is running, e.g. roslaunch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 fcu_protocol:=v1.0"
    )

    rospy.init_node("apm_calibrate", anonymous=True)

    apm.set_apm_mode("MANUAL")
    apm.set_parameter("TARGET_MODE", 3)

    apm.set_parameter("RUDDER_MIN", 1500)
    apm.set_parameter("RUDDER_MID", 1500)
    apm.set_parameter("RUDDER_MAX", 1500)
    apm.set_parameter("SAIL_MIN", 1500)
    apm.set_parameter("SAIL_MID", 1500)
    apm.set_parameter("SAIL_MAX", 1500)
    apm.set_parameter("WIND_MID", 1500)
    apm.set_parameter("WIND_MIN", 1500)
    apm.set_parameter("WIND_MAX", 1500)

    calibration_data = {}
    calibration_data["RUDDER_MIN"] = _calibrate_servo(1300, ServoType.RUDDER, "left")
    calibration_data["RUDDER_MID"] = _calibrate_servo(1300, ServoType.RUDDER, "midship")
    calibration_data["RUDDER_MAX"] = _calibrate_servo(1300, ServoType.RUDDER, "right")

    calibration_data["SAIL_MIN"] = _calibrate_servo(1300, ServoType.SAIL, "left")
    calibration_data["SAIL_MID"] = _calibrate_servo(1300, ServoType.SAIL, "midship")
    calibration_data["SAIL_MAX"] = _calibrate_servo(1300, ServoType.SAIL, "right")

    wind_mid, wind_min, wind_max = _calibrate_wind_sensor()
    calibration_data["WIND_MIN"] = wind_min
    calibration_data["WIND_MID"] = wind_mid
    calibration_data["WIND_MAX"] = wind_max

    _save_calibration_data(calibration_data)



def _calibrate_servo(start_value: int, target_servo: ServoType, position: str) -> int:
    print(
        f"\nStarting {target_servo.name} {position} calibration with initial value {start_value}"
    )
    print("Use arrow keys to set the desired value and confirm with ENTER!")
    current_value = start_value
    while True:
        c = _getchar()
        if c == EOT or c == ETX:
            # CTRL-D or CTRL-C is pressed
            print("Exiting...")
            raise RuntimeError(
                f"Could not calibrate {target_servo.name} {position} due to user exit"
            )
        elif c == RET:
            print("Confirming value ", current_value)
            # Enter is pressed
            return current_value
        # Try to read arrow key
        elif c == ESC:
            # ASCII escape character: ^[
            if _getchar() == CSI:
                # Control sequence introducer: [
                x = _getchar()
                if x == UP:
                    # E.g. UP is send as three characters via terminal: '^[' + '[' + 'A'
                    current_value += 10
                elif x == DOWN:
                    current_value -= 10
                elif x == RIGHT:
                    current_value += 50
                elif x == LEFT:
                    current_value -= 50
                else:
                    continue
                _set_servo_value(target_servo, current_value)
                print(
                    f"Current {target_servo.name} value: {_get_servo_value(target_servo)}"
                )


def _getchar():
    fd = sys.stdin.fileno()
    attr = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, attr)


def _get_servo_value(target_servo: ServoType) -> int:
    return apm.get_rc_channel(target_servo.rc_channel)


def _set_servo_value(target_servo: ServoType, value) -> None:
    apm.set_rc_channel(target_servo.rc_channel, value)


def _calibrate_wind_sensor() -> Tuple[int, int, int]:
    print(
        "Wind vane calibration: Rotate a few time while pressing SPACE, then align from front and press ENTER!"
    )
    min_value = max_value = _get_wind_sensor_value()
    while True:
        current_value = _get_wind_sensor_value()
        print(f"Current wind sensor value: {current_value}")
        min_value = min(min_value, current_value)
        max_value = max(max_value, current_value)
        c = _getchar()
        if c == EOT or c == ETX:
            # CTRL-D or CTRL-C is pressed
            print("Exiting...")
            raise RuntimeError(f"Could not calibrate wind sensor due to user exit")
        elif c == RET:
            return current_value, min_value, max_value


def _get_wind_sensor_value() -> int:
    return apm.get_rc_channel(WIND_SENSOR_RC_CHANNEL)

def _save_calibration_data(calibration_data : Dict[str, int]):
    __location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))
    file_path = os.path.join(__location__,"calibration_data.json" )
    print(f"Saving calibration data to: {file_path}")
    with open(file_path, "w") as f:
        json.dump(calibration_data, f, ensure_ascii=False, indent=4)
