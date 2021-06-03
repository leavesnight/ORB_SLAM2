#!/bin/bash
#EUROCFILE=MH02easy
#CAMTYPE=StereoVIO
#SUBFILE=StereoVIO
CAMTYPEFOLDER=${CAMTYPE%VIO}
mkdir ~/dataset/EuRoC/$EUROCFILE/orbslam2/
mkdir ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
cp -r ~/tmp/zzh ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
cp -r ~/zzh/ORB_SLAM2/Examples/$CAMTYPEFOLDER/CameraTrajectory.txt ~/zzh/ORB_SLAM2/Examples/$CAMTYPEFOLDER/KeyFrameTrajectory.txt ~/zzh/ORB_SLAM2/Examples/$CAMTYPEFOLDER/KeyFrameTrajectoryIMU.txt ~/zzh/ORB_SLAM2/Examples/$CAMTYPEFOLDER/KeyFrameTrajectoryIMU_NO_FULLBA.txt ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/

