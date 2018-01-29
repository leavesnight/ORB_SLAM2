#!/bin/bash
#EUROCFILE=V203difficult
#EUROCFILE2=V203
cd ~/zzh/ORB_SLAM2/Examples/Stereo
./stereo_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/$EUROCFILE/mav0/cam0/data ~/dataset/EuRoC/$EUROCFILE/mav0/cam1/data ./EuRoC_TimeStamps/$EUROCFILE2.txt ~/dataset/EuRoC/$EUROCFILE/mav0/imu0/data.csv 
CAMTYPE=Stereo

