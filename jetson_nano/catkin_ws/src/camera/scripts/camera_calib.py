#!/usr/bin/env python3

import cv2 as cv # OpenCV library
import numpy as np
import glob, os
from datetime import datetime as dt
from pathlib import Path


camport = cv.CAP_ANY

home = Path.home()
now = dt.now()
folder_id = now.strftime("%d-%m-%Y_%H-%M-%S")
work_path = str(home) + "/isp-2022/jetson_nano/catkin_ws/src/camera/scripts/calib_samples/" + folder_id
os.mkdir(work_path)
os.mkdir(work_path + "/left")
os.mkdir(work_path + "/right")

print(work_path)
def make_shots(cam: int = camport):
    cap = cv.VideoCapture(cam)
    counter = 0

    while(True):
        _, frame = cap.read()
        cv.imshow("video feed", frame)
        if cv.waitKey(1) & 0xFF == ord('s'):
            counter += 1
            width_cutoff = frame.shape[1] // 2
            frame_left = frame[:, width_cutoff:] # left
            frame_right = frame[:, :width_cutoff] # right
            cv.imshow("right shot", frame_right)
            cv.imshow("left shot", frame_left)
            cv.imwrite(work_path + "/left/video_shot_" + str(counter) + ".png", frame_left)
            cv.imwrite(work_path + "/right/video_shot_" + str(counter) + ".png", frame_right)
            cv.waitKey(1000)
        if cv.waitKey(1) & 0xFF == ord('p'):
            print("x gedruckt")
            cv.destroyAllWindows()
            break


def cam_calib(chessboardSize: tuple = (8, 6), frameSize: tuple = (1280, 720), chess_size: int = 25):
    ################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

    size_of_chessboard_squares_mm = chess_size
    objp = objp * size_of_chessboard_squares_mm

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpointsL = [] # 2d points in image plane.
    imgpointsR = [] # 2d points in image plane.
    imagesLeft = sorted(glob.glob(work_path + "/left/*.png"))
    imagesRight = sorted(glob.glob(work_path + "/right/*.png"))

    print(imagesLeft)
    print(imagesRight)

    for imgLeft, imgRight in zip(imagesLeft, imagesRight):
        print("image da")

        imgL = cv.imread(imgLeft)
        imgR = cv.imread(imgRight)
        grayL = cv.cvtColor(imgL, cv.COLOR_BGR2GRAY)
        grayR = cv.cvtColor(imgR, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        retL, cornersL = cv.findChessboardCorners(grayL, chessboardSize, None)
        retR, cornersR = cv.findChessboardCorners(grayR, chessboardSize, None)

        # If found, add object points, image points (after refining them)
        if retL and retR == True:
            print("objectpoint")

            objpoints.append(objp)

            cornersL = cv.cornerSubPix(grayL, cornersL, (11,11), (-1,-1), criteria)
            imgpointsL.append(cornersL)

            cornersR = cv.cornerSubPix(grayR, cornersR, (11,11), (-1,-1), criteria)
            imgpointsR.append(cornersR)

            # Draw and display the corners
            cv.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
            cv.imshow('img left', imgL)
            cv.drawChessboardCorners(imgR, chessboardSize, cornersR, retR)
            cv.imshow('img right', imgR)
            cv.waitKey(1000)


    cv.destroyAllWindows()

    ############## CALIBRATION #######################################################

    retL, cameraMatrixL, distL, rvecsL, tvecsL = cv.calibrateCamera(objpoints, imgpointsL, frameSize, None, None, flags=cv.CALIB_FIX_K3)
    heightL, widthL, channelsL = imgL.shape
    newCameraMatrixL, roi_L = cv.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))

    retR, cameraMatrixR, distR, rvecsR, tvecsR = cv.calibrateCamera(objpoints, imgpointsR, frameSize, None, None, flags=cv.CALIB_FIX_K3)
    heightR, widthR, channelsR = imgR.shape
    newCameraMatrixR, roi_R = cv.getOptimalNewCameraMatrix(cameraMatrixR, distR, (widthR, heightR), 1, (widthR, heightR))



    ########## Stereo Vision Calibration #############################################

    flags = 0
    flags |= cv.CALIB_FIX_INTRINSIC
    flags |= cv.CALIB_SAME_FOCAL_LENGTH
    # Here we fix the intrinsic camara matrixes so that only Rot, Trns, Emat and Fmat are calculated.
    # Hence intrinsic parameters are the same 

    criteria_stereo= (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # This step is performed to transformation between the two cameras and calculate Essential and Fundamenatl matrix
    retStereo, newCameraMatrixL, distL, newCameraMatrixR, distR, rot, trans, essentialMatrix, fundamentalMatrix = cv.stereoCalibrate(objpoints, imgpointsL, imgpointsR, newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], criteria_stereo, flags)

    ########## Stereo Rectification #################################################

    rectifyScale = 1
    rectL, rectR, projMatrixL, projMatrixR, Q, roi_L, roi_R= cv.stereoRectify(newCameraMatrixL, distL, newCameraMatrixR, distR, grayL.shape[::-1], rot, trans, rectifyScale,(0,0), alpha=0.0)

    stereoMapL = cv.initUndistortRectifyMap(newCameraMatrixL, distL, rectL, projMatrixL, grayL.shape[::-1], cv.CV_16SC2)
    stereoMapR = cv.initUndistortRectifyMap(newCameraMatrixR, distR, rectR, projMatrixR, grayR.shape[::-1], cv.CV_16SC2)

    print("Saving parameters!")
    cv_file = cv.FileStorage('stereoMap.xml', cv.FILE_STORAGE_WRITE)

    cv_file.write('stereoMapL_x',stereoMapL[0])
    cv_file.write('stereoMapL_y',stereoMapL[1])
    cv_file.write('stereoMapR_x',stereoMapR[0])
    cv_file.write('stereoMapR_y',stereoMapR[1])

    cv_file.release()

def main():
    make_shots()
    cam_calib()


if __name__ == '__main__':
    main()
