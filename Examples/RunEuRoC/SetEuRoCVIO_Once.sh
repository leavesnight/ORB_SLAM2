#!/bin/bash
OFFSET=-0.2
EUROCFILE=V203difficult
EUROCFILE2=V203
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCVIO.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=VIO/NoLoopMore_-0.2
source ./EvaluateEuRoC_Evaluate.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=StereoVIO
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

