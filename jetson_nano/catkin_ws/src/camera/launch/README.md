# camera project ROS lanuchfiles

runnable by ``roslaunch camera <launchfile>``

## cam_corr_depth.launch

Runs the camera publisher node and uses raw images for ROS darknet (YOLO) and the corrected frames for the depth map generation.

![Image](rosgraph.svg "graph of cam_corr_depth")

## calibration.launch

Runs the calibration process (more infoirmation inside README.md in /scripts)