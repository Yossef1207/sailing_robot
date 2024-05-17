#!/bin/sh
echo 'starting logging-node...'
screen -dmS logging bash -c 'cd /isp-2022/jetson_nano/catkin_ws/src/loggingNode; rosrun loggingNode logging'
sleep 20
echo 'starting roscore...'
screen -dmS roscore bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws; roscore'
sleep 5
echo 'starting mavros...'
screen -dmS mavros bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws; roslaunch mavros apm.launch fcu_url:=serial:///dev/ttyACM0:115200 fcu_protocol:=v1.0 gcs_url:=udp://@localhost'
sleep 20
#echo 'starting rudder-node...'
#screen -dmS rudder bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/src/boat_control/src/boat_control/; rosrun boat_control rudder'
#sleep 10
#echo 'starting sailing-node...'
#screen -dmS sailing bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/src/boat_control/src/boat_control; rosrun boat_control sailsetter'
#sleep 10
echo 'quick calibration...'
screen -dmS calibration bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/src/boat_control/src/boat_control/scripts; rosrun boat_control quick_calibrate'
sleep 5
echo 'starting gps...'
screen -dmS gps bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/src/boat_control/src/boat_control/scripts; rosrun boat_control gps'
sleep 5
echo 'starting gui...'
screen -dmS gui bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/src/boat_control/src/boat_control/scripts; rosrun boat_control gui'
sleep 5
echo 'starting windsensor...'
screen -dmS windsensor bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/src/boat_control/boat_control/src; rosrun boat_control windsensor.py'
sleep 5
echo 'starting control-node...'
screen -dmS control bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/src/boat_control/scripts/; rosrun boat_control controller_easy'
sleep 5
echo 'starting camera-node...'
screen -dmS camera bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/; roslaunch camera camera_py.launch'
sleep 5
echo 'starting yolo-mediator...'
screen -dmS yoloMediator bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/; roslaunch camera yolo_mediator.launch'
sleep 5
echo 'starting yolov3...'
screen -dmS yolo bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/; roslaunch darknet_ros yolo_v3_tiny.launch'
sleep 5
#Caused to Memory: echo 'starting yolov7...'
#screen -dmS yolo bash -c 'cd /isp-2022/jetson_nano/catkin_ws/; roslaunch yolov7_ros yolov7.launch'
#sleep 5
echo 'starting sort...'
screen -dmS sort bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/; roslaunch sort_track sort.launch'
sleep 5
echo 'depth-calculation...'
screen -dmS depthCalculator bash -c 'cd ~/isp-2022/jetson_nano/catkin_ws/; rosrun camera depth.py'
sleep 5
echo 'Finished Starting-Script'
