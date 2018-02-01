#!/bin/bash
#OURFILE=~/test_save/test_001difficult2
#OURFILE=~/dataset/test_003_corridor
OURFILE=~/dataset/test_outdoor
#EUROCFILE2=V203
cd ~/zzh/ORB_SLAM2/Examples/RGB-D
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_sd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/EncSensor.txt
./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_sd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/odometrysensor.txt
CAMTYPE=RGBD

#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_sd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9 0 $OURFILE/EncSensor.txt
#./rgbd_tum ../../Vocabulary/ORBvoc.bin ./kinect2_sd.yaml $OURFILE $OURFILE/associate.txt $OURFILE/IMUSensor.txt 9
