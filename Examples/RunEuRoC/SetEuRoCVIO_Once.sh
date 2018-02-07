#!/bin/bash
OFFSET=0.2
EUROCFILE=MH05difficult
EUROCFILE2=MH05
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCVIO.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=VIO/NoLoopMore_0.2
source ./EvaluateEuRoC_Evaluate.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=StereoVIO
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

