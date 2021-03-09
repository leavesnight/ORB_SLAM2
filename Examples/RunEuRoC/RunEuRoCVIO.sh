#!/bin/bash
#EUROCFILE=V203difficult
#EUROCFILE2=V203
CAMTYPE="Stereo"
if [[ $1 != "" ]]; then
    CAMTYPE=$1
fi
echo "CAMTYPE="$CAMTYPE
cd ~/zzh/ORB_SLAM2/Examples/$CAMTYPE
if [[ $CAMTYPE == "Stereo" ]]; then
    ./stereo_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/$EUROCFILE/mav0/cam0/data ~/dataset/EuRoC/$EUROCFILE/mav0/cam1/data ./EuRoC_TimeStamps/$EUROCFILE2.txt ~/dataset/EuRoC/$EUROCFILE/mav0/imu0/data.csv
elif [[ $CAMTYPE == "Monocular" ]]; then
    ./mono_euroc ../../Vocabulary/ORBvoc.bin ./EuRoC_VIO.yaml ~/dataset/EuRoC/$EUROCFILE/mav0/cam0/data ./EuRoC_TimeStamps/$EUROCFILE2.txt ~/dataset/EuRoC/$EUROCFILE/mav0/imu0/data.csv
fi

