# Sailing boat hardware stack setup

## Jetson OS Setup

The easiest way to flash the operating system to the SD card is using the [Nvidia SDK manager](https://developer.nvidia.com/drive/sdk-manager). The SDK manager offers the possibility to select what components should be installed. I would recommend a full installation, as you won´t be able to add those components later on. For the installation process you will need a micro USB cable and possibly an additional jumper, if you are setting up the Jetson for the first time. Just follow the steps described in the SDK manager.

## Jetson Software Setup

Clone the GitLab repository:
```
git clone https://collaborating.tuhh.de/c00mtec2/isp-2022.git
```

For setting up the software on the Jetson there is a script "system_setup.sh". It can be called like this:

```
cd isp-2022/
./system-setup.sh [--general][--yolov3][--yolov7][--yoeo]
```
Always start by doing the general setup. The system will reboot once it is done.

After the reboot you can call the script again to install the other packages.

The "**yolov3**" and "**yoeo**" options are installing YoloV3 and Yoeo as they were set up in summer term 22.

"**yolov7**" installs packages for YoloV7, Sort and DeepSort, as set up in summer term 23.
This option might throw an error due to memory issues. Just call it a second time once it terminated and it should finish smoothly. The full installation will take multiple hours.

## Flashing the Arduino Nano

There is a script for flashing the Arduino Nano which connects the wind sensors to the APM board. It is called like this:

```
./flashArduino.sh <device>
```
Again, you might need to make it executable first. Under device it needs to be specified under which name the Arduino is available. For example "USB0". You can find it using the command
```
arduino-cli board list
```
under "Port".

## Flashing the APM board

The APM is flashed from the Virtual Machine. Make sure the Virtual Machine and the Jetson are in the same network. On the VM, please go to "isp-2022/apm_boat/APMboat".
There exist multiple scripts for uploading the code to the APM board. First call:
```
./initJetson.sh <Jetson IP>
```
This script only has to be executed once after a fresh Jetson setup.

To create a HEX file from the APM code, the makeHex script needs to be called:
```
./makeHex.sh
```
If anything goes wrong, the following script might help:
```
./cleanHex.sh
```
After that the code can be uploaded to the APM by executing the following script:
```
./uploadHexToBoatJetson.sh <Jetson IP>
```

## Configuring the APM

Some internal APM parameters need to be changed. Also, the compass should be calibrated. The best tool for this is [MissionPlanner](https://firmware.ardupilot.org/Tools/MissionPlanner/archive/). There are problems with the latest version, use version 1.3.74 for full support.

Inside MissionPlanner connect to the APM board. Go to CONFIG -> Full Parameter List.
Adjust the following parameters:
- AHRS_ORIENTATION 0 -> 4
- COMPASS_EXTERNAL 0 -> 1
- COMPASS_ORIENT 0 -> 4
- COMPASS_USE 0 -> 1
- RC5_FUNCTION 0 -> -1
- RC6_FUNCTION 0 -> -1

## Calibrating the APM

The accelerometer and the compass should be calibrated. Go to SETUP -> Basis Hardware. First go to "Calibration accelerometer" (Kalibrierung Beschleunigungsensor) and follow the calibration steps. After that go to "Compass" and choose 'Live'-Calibration. The compass calibration should be done outside, because we need a strong GPS signal. Wait until you see something like "3D GPS fix" in the top left image under the "DATA" tab. A video on how to do the calibration is linked in MissionPlanner and various others can be found on Youtube.