#!/bin/bash
#EUROCFILE=MH02easy
#CAMTYPE=Stereo
#SUBFILE=StereoVIO
mkdir ~/dataset/EuRoC/$EUROCFILE/orbslam2/
mkdir ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
cp -r ~/tmp/zzh ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/
cp -r ~/zzh/ORB_SLAM2/Examples/$CAMTYPE/CameraTrajectory.txt ~/zzh/ORB_SLAM2/Examples/$CAMTYPE/KeyFrameTrajectory.txt ~/zzh/ORB_SLAM2/Examples/$CAMTYPE/KeyFrameTrajectoryIMU.txt ~/zzh/ORB_SLAM2/Examples/$CAMTYPE/KeyFrameTrajectoryIMU_NO_FULLBA.txt ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/

