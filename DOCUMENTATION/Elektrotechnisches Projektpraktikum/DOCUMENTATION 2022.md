# General

This markdown file describes the different components that have been added during the "Elektrotechnisches Projektpraktikum" 2022.
The goal of this project was to implement a control strategy for the camera gimbal in order to stabilize the camera. Only the files in **apm_boat/APMboat** have been edited. The folder contains one more subfolder called **Active Vision 2022** where the group left a text file with some commands that they used frequently.

# Functions

The gimbal control functions can be found in the **GimbalControl.cpp** file. Here a quick overview:
- **in_servo_range**: Checks if a target value is between a min and a maximum value. If it is outside of the range, return the min/max, otherwise the target value.
- **array_mean**: Calculates the mean over all values in an array.
- **shift_array**: Shifts the values in an array by one position.
- **angle_to_PWM**: Converts a target angle to the corresponding PWM value for a servo.
- **gimbal_adjust_roll**: Calculates a new output value for the roll servo and outputs it.
- **gimbal_adjust_yaw**: Calculates a new value for the yaw servo and outputs it.

The functions belong to the Rover class, which is the main class of the project. Therefore, they also have to be defined in the **Rover.h** header file. This is done in line **390 to 395**.

# Parameters

Parameters that do not need to be accessed from outside (through Mavlink) can be defined as member variables from the Rover class. This is also done in the **Rover.h** file. In this project, two variables **last_roll_servo_PWM_values** and **last_yaw_servo_PWM_values** have been defined (line 114 & 115). They store the last three calculated roll/yaw PWM values. To generate the output for either of the servos, the mean over the last three values is calculated. This leads to a smoother output signal.

Parameters that should be accessible from the outside, can be defined in the **Parameters.cpp/Parameters.h**. The following new parameters have been defined:
- camera_roll_min (PWM value)
- camera_roll_mid (PWM value)
- camera_roll_max (PWM value)
- roll_range (degrees)
- camera_yaw_min (PWM value)
- camera_yaw_mid (PWM value)
- camera_yaw_max (PWM value)
- yaw_range (degrees)
- cam_yaw_target (degrees, in heading frame)

They are defined at three locations:
- Parameters.h line 76 to 85
- Parameters.h line 350 to 359
- Parameters.cpp line 666 to 676

# Scheduler

To frequently call the **gimbal_adjust_roll** and **gimbal_adjust_yaw** function they had to be added to the scheduler defined in the **APMboat.cpp** file in line 48ff.

# Helper Scripts

There are three helper scripts for uploading the code to the APM board:
- **makeHex.sh** to build the HEX file
- **cleanHex.sh** to clean the HEX file
- **uploadHexToBoat.sh** to upload the HEX file to the boat

However, the third script was written to upload the HEX file via a RaspberryPi (old boat version). The new version with the Jetson Nano uses slightly different paths, therefore I added three more helper scripts:
- **uploadJetson.sh**: Script that will be copied to the Jetson and executed from there. Does not need to be called on its own.
- **initJetson.sh** to create new folders on the Jetson and copy the uploadJetson.sh script. Does only need to be called once.
- **uploadHexToBoatJetson.sh** to upload a new Hex file to the APM via the Jetson.

So for a new Jetson, the workflow would look like this:

Call initJetson once (replace xxx with the correct IP for your Jetson):
```
./initJetson.sh 192.168.0.xxx
```
Install avrdude on the Jetson:
```
ssh nano@192.168.0.xxx
sudo apt-get install avrdude
```
Every time you want to upload a new HEX file:
```
./makeHex.sh
./uploadHexToBoatJetson.sh 192.168.0.xxx
```
If you run into errors, you can try to clean the old HEX file by calling:
```
./cleanHex.sh
```
