#!/bin/bash
OFFSET=0.
EUROCFILE=V103difficult
EUROCFILE2=V103
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
#CAMTYPE=Monocular
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

