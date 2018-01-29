#!/bin/bash
#V1XX
OFFSET=0.
EUROCFILE=V101easy
EUROCFILE2=V101
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCVIO.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=VIO/NoLoopMore
source ./EvaluateEuRoC_Evaluate.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=StereoVIO
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=V102medium
EUROCFILE2=V102
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCVIO.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=VIO/NoLoopMore
source ./EvaluateEuRoC_Evaluate.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=StereoVIO
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=V103difficult
EUROCFILE2=V103
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCVIO.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=VIO/NoLoopMore
source ./EvaluateEuRoC_Evaluate.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=StereoVIO
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

#V2XX
OFFSET=-0.2
EUROCFILE=V201easy
EUROCFILE2=V201
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCVIO.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=VIO/NoLoopMore_-0.2
source ./EvaluateEuRoC_Evaluate.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=StereoVIO
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=V202medium
EUROCFILE2=V202
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCVIO.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=VIO/NoLoopMore_-0.2
source ./EvaluateEuRoC_Evaluate.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=StereoVIO
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

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

