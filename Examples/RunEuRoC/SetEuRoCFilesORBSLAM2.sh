#!/bin/bash
#V1XX
OFFSET=0.
EUROCFILE=V101easy
EUROCFILE2=V101
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=V102medium
EUROCFILE2=V102
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=V103difficult
EUROCFILE2=V103
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

#V2XX
OFFSET=-0.2
EUROCFILE=V201easy
EUROCFILE2=V201
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=V202medium
EUROCFILE2=V202
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=V203difficult
EUROCFILE2=V203
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

#MHXX
OFFSET=0.2
EUROCFILE=MH01easy
EUROCFILE2=MH01
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=MH02easy
EUROCFILE2=MH02
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=MH03medium
EUROCFILE2=MH03
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=MH04difficult
EUROCFILE2=MH04
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

EUROCFILE=MH05difficult
EUROCFILE2=MH05
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./RunEuRoCORBSLAM2.sh
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
SUBFILE=$CAMTYPE
source ./EvaluateEuRoC_Copy.sh
source ./EvaluateEuRoC_Evaluate.sh

