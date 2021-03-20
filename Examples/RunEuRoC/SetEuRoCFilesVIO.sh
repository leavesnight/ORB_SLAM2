#!/bin/bash
#V1XX
CAMTYPE="Stereo"
if [[ $1 != ""  ]]; then
    CAMTYPE=$1
fi

OFFSET=0.
EUROCFILE=V101easy
EUROCFILE2=V101
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./SetEuRoCVIO_Once.sh $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET &

EUROCFILE=V102medium
EUROCFILE2=V102
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./SetEuRoCVIO_Once.sh $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET &

OFFSET=-0.2
EUROCFILE=V201easy
EUROCFILE2=V201
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./SetEuRoCVIO_Once.sh $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET &

EUROCFILE=V202medium
EUROCFILE2=V202
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./SetEuRoCVIO_Once.sh $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET &

wait

EUROCFILE=V203difficult
EUROCFILE2=V203
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./SetEuRoCVIO_Once.sh $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET

OFFSET=0.2
EUROCFILE=MH04difficult
EUROCFILE2=MH04
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./SetEuRoCVIO_Once.sh $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET

OFFSET=0.
EUROCFILE=V103difficult
EUROCFILE2=V103
cd ~/zzh/ORB_SLAM2/Examples/RunEuRoC
source ./SetEuRoCVIO_Once.sh $CAMTYPE $EUROCFILE $EUROCFILE2 $OFFSET

SUBFILE=$CAMTYPE"VIO"

EUROCFILE=V101easy
EUROCFILE2=V101
echo "dataset_name=$EUROCFILE"
echo "result_ate.txt="
cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate.txt
EUROCFILE=V102medium
EUROCFILE2=V102
echo "dataset_name=$EUROCFILE"
echo "result_ate.txt="
cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate.txt
EUROCFILE=V201easy
EUROCFILE2=V201
echo "dataset_name=$EUROCFILE"
echo "result_ate.txt="
cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate.txt
EUROCFILE=V202medium
EUROCFILE2=V202
echo "dataset_name=$EUROCFILE"
echo "result_ate.txt="
cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate.txt
EUROCFILE=V203difficult
EUROCFILE2=V203
echo "dataset_name=$EUROCFILE"
echo "result_ate.txt="
cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate.txt
EUROCFILE=MH04difficult
EUROCFILE2=MH04
echo "dataset_name=$EUROCFILE"
echo "result_ate.txt="
cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate.txt
EUROCFILE=V103difficult
EUROCFILE2=V103
echo "dataset_name=$EUROCFILE"
echo "result_ate.txt="
cat ~/dataset/EuRoC/$EUROCFILE/orbslam2/$SUBFILE/result_ate.txt

exit

