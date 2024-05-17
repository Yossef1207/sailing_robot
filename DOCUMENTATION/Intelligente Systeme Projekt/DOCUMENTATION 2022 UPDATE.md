# OS setup

The intelligent systems group in 2022 used an unofficial Xubuntu 20.04 as their operating system on the Jetson Nano in order to make use of the (at that time) latest ROS version. However, this generally leads to more difficulties than advantages. Therefore, I set up their work on the official Jetpack (4.6.3) Ubuntu. The setup and necessary changes to the files left by the student group are documented in this markdown file.

## SDK manager

The easiest way to flash the operating system to the SD card is using the [Nvidia SDK manager](https://developer.nvidia.com/drive/sdk-manager). The SDK manager offers the possibility to select what components should be installed. I would recommend a full installation, as you won´t be able to add those components later on. For the installation process you will need a micro USB cable and possibly an additional jumper, if you are setting up the Jetson for the first time. Just follow the steps described in the SDK manager.

# Software Setup

First apply general updates by calling:
```
sudo apt update && sudo apt upgrade
```

## ROS installation

Follow the guide on the [official ROS page](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS melodic (desktop full).

## Mavlink/Mavros installation

Now Mavlink and Mavros can be installed. These packages are needed for communication with the APM board.
```
sudo apt-get install ros-melodic-mavlink ros-melodic-mavros ros-melodic-mavros-extras
```
The installation of geographic lib datasets is also necessary:
```
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash install_geographiclib_datasets.sh
sudo rm install_geographiclib_datasets.sh
```
You might need to add the user to the dialout group, in order to have permession to access the ports.
```
sudo adduser $USER dialout
```
Reboot for the changes to take effect. Now you can test the installation by running:
```
roslaunch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 fcu_protocol:=v1.0
```
It should show "Ready to drive" at some points and then you will be able to get and set parameters, e.g.:
```
rosrun mavros mavparam set RUDDER_MIN 100
```
```
rosrun mavros mavparam get RUDDER_MIN
```

## YOEO installation

I recommend installing YOEO from the official [GIT repository](https://github.com/bit-bots/YOEO).
For installation, python >3.8 is required, which still needs to be installed. We will use pyenv, a tool for managing multiple python versions.

### Pyenv installation

Install build dependencies:
```
sudo apt update; sudo apt install build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev curl \
libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev
```
Run the automatic installer:
```
curl https://pyenv.run | bash
```
Execute the following commands to add some configuration lines to the ~/.bashrc and ~/.profile file:
```
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init -)"' >> ~/.bashrc
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.profile
echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.profile
echo 'eval "$(pyenv init -)"' >> ~/.profile
```
Restart the shell:
```
exec "$SHELL"
```
We will use the latest python 3.8 version, which is 3.8.16:
```
pyenv install 3.8.16
```
For more details on how to use pyenv, have a look at the [official GIT](https://github.com/pyenv/pyenv).

### Cmake installation
In order to install YOEO you will need to upgrade Cmake to a version >=3.22. This needs to be done from source, please follow the [official guide](https://cmake.org/install/). The installation has been successfully tested with version 3.26.
The installation process will take quite some time (> 1h).

### GCC/G++ upgrade
Additionally, you will need to upgrade GCC/G++.
```
sudo apt install software-properties-common
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt install gcc-9 g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 1 --slave /usr/bin/g++ g++ /usr/bin/g++-9
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 2 --slave /usr/bin/g++ g++ /usr/bin/g++-7
```
You will now be able to select a gcc version by executing:
```
sudo update-alternatives --config gcc
```
Select version 9 for installing YOEO.

### Clone, install and test
Clone the GIT:
```
git clone https://github.com/bit-bots/YOEO
cd YOEO/
```
Set the python version to 3.8. ATTENTION: After calling this command, python 3.8 will automatically be activated in this folder. So calling pip commands from here will modify your python 3.8, not the global python 3.6!
```
pyenv local 3.8.16
```
And finally, install:
```
pip3 install poetry --user
PATH=$PATH:/home/nano/.local/bin
poetry install
```
To test the installation, you can make predictions on the sample images:
```
cd weights
./download_weights.sh
poetry run yoeo-detect --images data/samples/
```

## ROS setup

First clone the Git repository:
```
cd
git clone https://collaborating.tuhh.de/c00mtec2/isp-2022.git
```
The catkin workspace can be found under isp-2022/jetson_nano/catkin_ws.


In the file "/opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake" change the line
```
set(_include_dirs "include;/usr/include;/usr/include/opencv")
```
to
```
set(_include_dirs "include;/usr/include;/usr/include/opencv4")
```
This change is required for darknet.

Make sure to switch back to gcc/g++ 7. Also, make sure you are not in the YOEO folder, as we want to use the system python, not the one installed for YOEO.

Install some packages:
```
sudo apt-get install ros-melodic-ros-numpy python3-pip
pip3 install pyyaml
pip3 install rospkg
```

Now build darknet. Therefore, we first need to edit some path variables and create a symlink.
```
export CPATH=/usr/local/cuda-10.2/targets/aarch64-linux/include:$CPATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/targets/aarch64-linux/lib:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda-10.2/bin:$PATH
export LIBRARY_PATH=/usr/local/cuda/lib64:$LIBRARY_PATH
sudo ln -s /usr/local/cuda/lib64/libcudart.so /usr/lib/libcudart.so
```
After that, go into the darknet folder and call make.
```
cd isp-2022/jetson_nano/catkin_ws/src/darknet_ros/darknet
make
```
Finally, we can make the catkin workspace.
```
cd isp-2022/jetson_nano/catkin_ws
./release-build.sh
echo 'source /home/nano/isp-2022/jetson_nano/catkin_ws/devel/setup.bash' >> ~/.bashrc
```
The nodes should now work as described in the original documentation.

## File changes

Some files had to be changed in order to make everything work with the Jetpack setup. The changed are already applied to the files in the GIT. They are only listed here for reasons of documentation.

### ros_numpy

The camera nodes (catkin_ws/src/camera/scripts) are using the ROS package cv_bridge to convert between the ROS image format and numpy images. However, the nodes are executed with python3 and the standard cv_bridge installation only works with python2. Even though there are exist approaches to install cv_bridge with python3 support, it is much simpler to use the ros_numpy package instead.

The cv_bridge package is used in the following files:
- camera_corr.py
- camera_depth.py
- camera_pub.py
- yolo_mediator.py

In all of these files, replace:
```
from cv_bridge import CvBridge
```
with
```
import ros_numpy
```
and delete the line where a bridge object is created:
```
<bridge_name> = CvBridge()
```

After that replace all occurrences of
```
<bridge_name>.imgmsg_to_cv2(<ros_data>)
```
with
```
ros_numpy.numpify(<ros_data>)
```
and all occurrences of
```
<bridge_name>.cv2_to_imgmsg(<np_data>, encoding=<encoding>)
```
with
```
ros_numpy.msgify(Image, <np_data>, encoding=<encoding>)
```

### camera/scripts/camera_pub.py

Default values for "sample_rate" and "verbose" are missing in the camera_pub node. Change these lines:
```
rate = rospy.Rate(rospy.get_param('sample_rate'))
```
```
verbose = rospy.get_param('verbose')
```
such that they look like this:

```
rate = rospy.Rate(rospy.get_param('sample_rate', 10))
```
```
verbose = rospy.get_param('verbose', True)
```

### camera/launch/cam_corr_depth.launch

The path to the darknet launch file needs to be adjusted. Change line 14 to:

```
  <include file="$(find darknet_ros)/launch/darknet_ros.launch" />
```

### boat_control/src/boat_control/apm.py

The student group used an older Mavlink/Mavros version, where the "OverrideRCIn" message contained 8 values, while it contains 18 values in newer versions. We do not need theses additional values, so we are just adding dummy values in order to match the message size. In line 43:
```
channels += ([0]*10)
```
is added to extend the channel list by 10 zeros.

### darknet_ros/darknet/src/convolutional_layer.c

There is a compatibility issue between cuDNN v8 and darknet. A fix can be found [here](https://forums.developer.nvidia.com/t/object-detection-on-a-webcam-with-yolo/156431/7). Only apply the changes mentioned under "Fix the compatible issue for cuDNN v8."!