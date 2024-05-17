#!/bin/bash
set -e

CALLED_CORRECTLY=false

while getopts ":general-:yolov3-:yolov7-:yoeo-:" opt; do
	case ${opt} in
		-)
			case ${OPTARG} in
				"general")
					CALLED_CORRECTLY=true
					# upgrade and install ros
					cd
					sudo apt update && sudo apt upgrade -y
					sudo apt install nano -y
					sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
					sudo apt install curl -y
					curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
					sudo apt update
					sudo apt install ros-melodic-desktop-full -y
					echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
					source ~/.bashrc
					sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
					sudo rosdep init
					rosdep update
					# mavros specific installs
					sudo apt-get install ros-melodic-mavlink ros-melodic-mavros ros-melodic-mavros-extras -y
					wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
					sudo bash install_geographiclib_datasets.sh
					sudo rm install_geographiclib_datasets.sh
					sudo adduser $USER dialout
					sudo sed -i 's+set(_include_dirs "include;/usr/include;/usr/include/opencv")+set(_include_dirs "include;/usr/include;/usr/include/opencv4")+g' /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake
					sudo apt-get install ros-melodic-ros-numpy python3-pip python-pip -y
					pip3 install pyyaml rospkg
					# set cuda paths
					echo "export CPATH=/usr/local/cuda-10.2/targets/aarch64-linux/include:\$CPATH" >> ~/.bashrc
					echo "export LD_LIBRARY_PATH=/usr/local/cuda-10.2/targets/aarch64-linux/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc
					echo "export PATH=/usr/local/cuda-10.2/bin:\$PATH" >> ~/.bashrc
					echo "export LIBRARY_PATH=/usr/local/cuda/lib64:\$LIBRARY_PATH" >> ~/.bashrc
					sudo ln -s /usr/local/cuda/lib64/libcudart.so /usr/lib/libcudart.so
					echo 'source /home/nano/isp-2022/jetson_nano/catkin_ws/devel/setup.bash' >> ~/.bashrc
					# stuff for flashing APM/Arduino
					sudo apt install avrdude -y
					curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
					echo "export PATH=/home/nano/bin:\$PATH" >> ~/.bashrc
					source ~/.bashrc
					# new packages for stuff from summer term 23
					sudo apt install screen -y
					sudo apt install libxslt-dev -y
					pip install dronekit
					pip install pymavlink==2.3.6
					pip3 install dronekit pyserial flask
					cd /home/nano/isp-2022/jetson_nano/catkin_ws
					catkin_make
					sudo reboot now
					;;
				# installs components needed for yolov3
				"yolov3")
					CALLED_CORRECTLY=true
					cd /home/nano/isp-2022/jetson_nano/catkin_ws/src/darknet_ros/darknet
					make
					cd /home/nano/isp-2022/jetson_nano/catkin_ws
					./release-build.sh
					cd /home/nano/isp-2022/jetson_nano/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights/
					wget https://pjreddie.com/media/files/yolov3-tiny.weights
					;;
				# installs components needed for yolov7 and deepsort
				"yolov7")
					CALLED_CORRECTLY=true
					cd
					git clone https://github.com/wjakob/tbb.git || true
					cd tbb/build
					cmake ..
					make -j
					sudo make install
					sudo apt install llvm-10 -y
					export LLVM_CONFIG=/usr/bin/llvm-config-10
					pip3 install llvmlite
					pip3 install Cython==0.29.36
					pip3 install numba
					sudo apt-get install gfortran -y
					pip3 install scipy colorama scikit-image
					pip3 install scikit-learn==0.22.2.post1
					pip3 install filterpy
					pip3 install protobuf==3.19.6
					# install tensorflow
					sudo -H pip3 install --extra-index-url https://developer.download.nvidia.com/compute/redist/jp/v461 tensorflow==1.15.5+nv22.01
					sudo apt-get install libomp5 -y
					sudo apt-get install libopenblas-dev -y
					# install pytorch and torchvision
					pip3 install /home/nano/isp-2022/jetson_nano/catkin_ws/src/yolov7-ros/torch-1.10.0a0+git36449ea-cp36-cp36m-linux_aarch64.whl
					sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev -y
					cd
					git clone --branch release/0.11 https://github.com/pytorch/vision torchvision
					cd torchvision
					export BUILD_VERSION=0.11.0
					python3 setup.py install --user
					pip3 install tqdm
					pip3 install seaborn
					cd /home/nano/isp-2022/jetson_nano/catkin_ws/src/yolov7-ros
					mkdir weights
					cd weights
					wget https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7-tiny.pt
					# make sure numpy is at the correct version (1.19.5 is not compatible)
					pip3 install numpy==1.19.4
				;;
				# installs yoeo
				"yoeo")
					CALLED_CORRECTLY=true
					sudo apt update
					sudo apt install build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev curl libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev -y
					curl https://pyenv.run | bash
					echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
					echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
					echo 'eval "$(pyenv init -)"' >> ~/.bashrc
					echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.profile
					echo 'command -v pyenv >/dev/null || export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.profile
					echo 'eval "$(pyenv init -)"' >> ~/.profile
					source ~/.bashrc
					pyenv install 3.8.16
					wget https://github.com/Kitware/CMake/releases/download/v3.26.4/cmake-3.26.4.tar.gz
					tar -xf cmake-3.26.4.tar.gz
					sudo rm cmake-3.26.4.tar.gz
					cd cmake-3.26.4/
					./bootstrap
					make
					sudo make install
					cd ..
					sudo rm -r cmake-3.26.4
					cd
					sudo apt install software-properties-common -y
					sudo add-apt-repository ppa:ubuntu-toolchain-r/test -y
					sudo apt install gcc-9 g++-9 -y
					sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 1 --slave /usr/bin/g++ g++ /usr/bin/g++-9
					sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 2 --slave /usr/bin/g++ g++ /usr/bin/g++-7
					sudo update-alternatives --set gcc /usr/bin/gcc-9
					git clone https://github.com/bit-bots/YOEO
					cd YOEO/
					pyenv local 3.8.16
					pip install poetry --user
					PATH=$PATH:/home/nano/.local/bin
					poetry install
					sudo update-alternatives --set gcc /usr/bin/gcc-7
					;;
			esac
	esac
done
if [ $CALLED_CORRECTLY = false ]; then
	echo "Usage:"
	echo "./system_setup.sh [--general][--yolov3][--yolov7][--yoeo]"
	echo ""
	echo "Please do the general setup before trying to install the other modules. Otherwise errors might occur."
fi
