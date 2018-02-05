#!/bin/bash
#OURFILE=~/dataset/Lab001easy
#OURFILE=~/dataset/Lab002medium
#OURFILE=~/dataset/Lab003difficult
OURFILE=~/dataset/Corridor001easy
#OURFILE=~/dataset/Corridor002medium
#OURFILE=~/dataset/Corridor003difficult
#OURFILE=~/dataset/Corridor004difficult
#OURFILE=~/dataset/Farm002medium
#EUROCFILE2=V203
cd ~/zzh/ORB_SLAM2/Examples/RGB-D
./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_sd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/EncSensor.txt
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_sd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/odometrysensor.txt
CAMTYPE=RGBD

#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_sd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9 0 $OURFILE/EncSensor.txt
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_sd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9
