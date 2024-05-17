# camera python scripts

## Camera calibration (camera_calib.py)

Takes shots from camera if key "s" is pressed. After all shots are taken, pressing "x" ends the image collecting process.
Afterwards a stereoMap.xml file is generated and saved. All selected images are saved in /left/video_shot_<index> and /right/video_shot_<index>.
If multiple cameras are connected, change camport variable to desired port number.

* explanations and tutorial of the calibration process [YoutTube: Camera Calibration](https://www.youtube.com/watch?v=3h7wgR5fYik)
* used source code from [niconielsen32: ComputerVision](https://github.com/niconielsen32/ComputerVision/tree/master/stereoVisionCalibration)
* mathematical details [OpenCV: Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

## Camera correction _deprecated_ (camera_corr.py)

Subscribes to *video_frames* ROS topic and corrects video feed by stereoMap.xml file. The video feed then is showed in its raw and corrected form splitted in left and right.

## Camera depthmap (camera_depth.py)

Subscribes to *camera/frame_corr_both* ROS topic and generates a depth map. The following 3 Parameters can be changed while the depth map is showed:

* number of disparities (from 1 to 17)
* block size (from 5 to 50)
* minimum number of disparities (from 5 to 25)

* more information about these maps at [learnopencv: depth perception](https://learnopencv.com/depth-perception-using-stereo-camera-python-c/)
* mathematical details [OpenCV: Depth Map](https://docs.opencv.org/3.4/dd/d53/tutorial_py_depthmap.html)

## Camera publisher (camera_pub.py)

Subscribes to */camera/frame_corr_both* and */camera/frame_both* ROS topics. The first one publishes corrected frames and the second one the raw uncorrected ones. The correction is done by the stereoMap.xml file.
