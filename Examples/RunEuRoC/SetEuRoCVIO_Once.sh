#!/bin/bash
OFFSET=0.2
EUROCFILE=MH05difficult
EUROCFILE2=MH05
if [[ $1 != ""  ]]; then
    CAMTYPE=$1
fi
if [[ $2 != "" && $3 != "" ]]; then
    EUROCFILE=$2
    ERUCOFILE2=$3
fi

cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCVIO.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
#SUBFILE=VIO/NoLoopMore_0.2
#source ./EvaluateEuRoC_Evaluate.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE"VIO"
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

