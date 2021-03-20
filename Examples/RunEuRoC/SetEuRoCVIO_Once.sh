#!/bin/bash
OFFSET=0.2
EUROCFILE=MH05difficult
EUROCFILE2=MH05
CAMTYPE="Stereo"
if [[ $1 != ""  ]]; then
    CAMTYPE=$1
fi
if [[ $2 != "" && $3 != "" ]]; then
    EUROCFILE=$2
    EUROCFILE2=$3
fi
if [[ $4 != "" ]]; then
    OFFSET=$4
fi

cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCVIO.sh $CAMTYPE $EUROCFILE $EUROCFILE2
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
#SUBFILE=VIO/NoLoopMore_0.2
#source ./EvaluateEuRoC_Evaluate.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE"VIO"
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

echo "dataset_name=$EUROCFILE"
echo "result_ate.txt="
cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate.txt

