#!/bin/sh
echo 'starting roscore...'
screen -dmS roscore bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws; roscore'
sleep 5
echo 'starting camera-node...'
screen -dmS camera bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/; roslaunch camera camera_py.launch'
sleep 5
echo 'starting yolo-mediator...'
screen -dmS yolo-mediator bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/; roslaunch camera yolo_mediator.launch'
sleep 5
echo 'starting yolov7...'
screen -dmS yolov7 bash -c 'cd /isp-2022/jetson_nano/catkin_ws/; roslaunch yolov7_ros yolov7.launch'
sleep 5
echo 'starting imageSaverNode...'
screen -dmS imageLogger bash -c 'cd /isp-2022/jetson_nano/catkin_ws/; rosrun beginner_tutorials listenerImage.py'
echo 'Finished Starting-Script'
