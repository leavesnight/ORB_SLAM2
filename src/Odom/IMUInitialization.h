//created by zzh, inspired by JingWang
#ifndef IMUINITIALIZATION_H
#define IMUINITIALIZATION_H

//zzh defined color cout, must after include opencv2
#define redSTR "\033[31m"
#define greenSTR "\e[32m"
#define blueSTR "\e[34m"
#define yellowSTR "\e[33m"
#define whiteSTR "\e[37m"

// #include <list>
#include <mutex>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

#include "OdomPreIntegrator.h"
// #include "KeyFrame.h"
// #include "Map.h"
#include "LocalMapping.h"

namespace ORB_SLAM2{

class KeyFrame;
class Map;
class LocalMapping;

//notice Get##Name() calls copy constructor when return
#define CREATOR_VAR_MUTEX(Name,Type,Suffix) \
  Type m##Suffix##Name;\
  std::mutex mMutex##Name;
#define CREATOR_GET(Name,Type,Suffix) \
  Type Get##Name(void){\
    unique_lock<mutex> lock(mMutex##Name);\
    return m##Suffix##Name;\
  }
#define CREATOR_SET(Name,Type,Suffix) \
  void Set##Name(Type value){\
    unique_lock<mutex> lock(mMutex##Name);\
    m##Suffix##Name=value;\
  }
#define CREATOR_VAR_MULTITHREADS(Name,Type,Suffix) \
private:\
  CREATOR_VAR_MUTEX(Name,Type,Suffix)\
public:\
  CREATOR_GET(Name,Type,Suffix)\
  CREATOR_SET(Name,Type,Suffix)\
private:
  
using namespace Eigen;
using namespace std;

class IMUInitialization{//designed for multi threads
  string mTmpfilepath;
  bool mbFullBA;//if execute full ba just after IMU Initialized
  double mdStartTime;//for reset
  //cv::Mat mRwiInit;//unused
  
  CREATOR_VAR_MULTITHREADS(SensorIMU,bool,b);//for auto reset judgement of this system, automatically check if IMU exists, for it needs initialization with a quite long period of tracking without LOST
  CREATOR_VAR_MULTITHREADS(VINSInited,bool,b)//if IMU initialization is over
  cv::Mat mGravityVec; // gravity vector in world frame
  std::mutex mMutexInitIMU;//for mGravityVec, improved by zzh
  //double mnVINSInitScale; //scale estimation for Mono, not necessary here
  
  CREATOR_VAR_MULTITHREADS(CopyInitKFs,bool,b)//for copying/cache KFs in IMU initialization thread avoiding KeyFrameCulling()
  
  //CREATOR_VAR_MULTITHREADS(UpdatingInitPoses,bool,b)//for last propagation in IMU Initialization to stop adding new KFs in Tracking thread, useless for LocalMapping is stopped
  CREATOR_VAR_MULTITHREADS(InitGBA,bool,b)//for last GBA(include propagation) required by IMU Initialization, LoopClosing always creates new GBA thread when it's true
  
  //like the part of LocalMapping
  CREATOR_VAR_MULTITHREADS(CurrentKeyFrame,KeyFrame*,p)//updated by LocalMapping thread
  CREATOR_VAR_MUTEX(Finish,bool,b)//checked/get by System.cc
  CREATOR_VAR_MUTEX(FinishRequest,bool,b)//requested/set by System.cc
  CREATOR_VAR_MULTITHREADS(Reset,bool,b)//for reset Initialization variables
  //const
  Map* mpMap;
  bool mbMonocular;
  LocalMapping* mpLocalMapper;//for Stop LocalMapping thread&&NeedNewKeyFrame() in Tracking thread
  
  bool TryInitVIO(void);
  cv::Mat SkewSymmetricMatrix(const cv::Mat &v){
      return (cv::Mat_<float>(3,3)<<0, -v.at<float>(2), v.at<float>(1),
				    v.at<float>(2), 0, -v.at<float>(0),
				    -v.at<float>(1), v.at<float>(0), 0);
  }
  void ResetIfRequested(){
    if(GetReset()){
      //reset relevant variables
      mdStartTime=-1;SetCurrentKeyFrame(NULL);//SetSensorIMU(false);
      SetVINSInited(false);//usually this 3 variables are false when LOST then this func. will be called
      SetInitGBA(false);//if it's true, won't be automatically reset
      
      SetReset(false);
    }
  }
  CREATOR_SET(Finish,bool,b)
  CREATOR_GET(FinishRequest,bool,b)
public:
  bool mbUsePureVision;//for pure-vision+IMU Initialization mode!
  
  IMUInitialization(Map* pMap,const bool bMonocular,const string& strSettingPath):mpMap(pMap),mbMonocular(bMonocular),mbFinish(true),mbFinishRequest(false),mbReset(false){
    mdStartTime=-1;mbSensorIMU=false;mpCurrentKeyFrame=NULL;
    mbVINSInited=false;
    mbCopyInitKFs=false;
    mbInitGBA = false;
    
    cv::FileStorage fSettings(strSettingPath,cv::FileStorage::READ);
    cv::FileNode fnStr=fSettings["test.InitVIOTmpPath"];
    if (!fnStr.empty()) fnStr>>mTmpfilepath;
    else cout<<"Nothing recorded for analysis!"<<endl;
    //load if Full BA just after IMU Initialized
    cv::FileNode fnFBA=fSettings["IMU.FullBA"];
    if (!fnFBA.empty()) mbFullBA=(int)fnFBA;
    else{
      mbFullBA=true;
      cout<<"Default execute FullBA!"<<endl;
    }
    //load mbUsePureVision
    cv::FileNode fnSize=fSettings["LocalMapping.LocalWindowSize"];
    if (fnSize.empty()){
      mbUsePureVision=true;
      cout<<redSTR"No LocalWindowSize, then don't enter VIORBSLAM2 or Odom(Enc/IMU) mode!"<<whiteSTR<<endl;
    }else{
      if ((int)fnSize<1){
	mbUsePureVision=true;
	cout<<blueSTR"mnLocalWindowSize<1, we use pure-vision+IMU Initialization mode!"<<whiteSTR<<endl;
      }else mbUsePureVision=false;
    }
  }
  
  void Run();
  
  cv::Mat GetGravityVec(void);
  
  CREATOR_GET(Finish,bool,b)
  CREATOR_SET(FinishRequest,bool,b)
  void RequestReset(){//blocking(3ms refreshing) mode, called by Tracking thread
    SetReset(true);
    for(;;){
      if (!GetReset()) break;//if mbReset changes from true to false, resetting is finished
      usleep(3000);
    }
  }
  void SetLocalMapper(LocalMapping* pLocalMapper){
    mpLocalMapper=pLocalMapper;
  }
};

class IMUKeyFrameInit{//a simple/base version of KeyFrame just used for IMU Initialization, not designed for multi threads
public://I think it belongs FramePoseBase
  const double mTimeStamp;//for ComputePreInt
  cv::Mat mTwc,mTcw;//for TryInitVIO()&OptimizeInitialGyroBias(),see (9) in VIORBSLAM paper
  //we don't save mTbc for it's constant
  
public:
  Vector3d mbg_,mba_;//bgj_bar,baj_bar: if changed, mIMUPreInt needs to be recomputed; unoptimized part of current defined mNavState
  IMUPreintegrator mOdomPreIntIMU;//including mlIMUData, for OptimizeInitialGyroBias()
  IMUKeyFrameInit* mpPrevKeyFrame;//but it's important for mOdomPreIntIMU computation && KeyFrameCulling()

  IMUKeyFrameInit(KeyFrame& kf);
  
  void ComputePreInt(){//0th frame don't use this function, mpPrevKeyFrame shouldn't be bad
    if (mpPrevKeyFrame==NULL) return;
#ifndef TRACK_WITH_IMU
    mOdomPreIntIMU.PreIntegration(mpPrevKeyFrame->mTimeStamp,mTimeStamp);
#else
    mOdomPreIntIMU.PreIntegration(mpPrevKeyFrame->mTimeStamp,mTimeStamp,mpPrevKeyFrame->mbg_,mpPrevKeyFrame->mba_);
#endif
  }
  
  //EIGEN_MAKE_ALIGNED_OPERATOR_NEW//for quaterniond in NavState
};

}

#endif