/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "OdomPreIntegrator.h"//zzh, for SetParam()

#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),
    mtimestampOdom(-1)
{
    // Load camera parameters from settings file
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    
    //load Tbc,Tbo refer the Jing Wang's configparam.cpp
    cv::FileNode fnT[2]={fSettings["Camera.Tbc"],fSettings["Camera.Tbo"]};
    Eigen::Matrix3d eigRtmp;
    for (int i=0;i<2;++i){
      eigRtmp<<fnT[i][0],fnT[i][1],fnT[i][2],fnT[i][4],fnT[i][5],fnT[i][6],fnT[i][8],fnT[i][9],fnT[i][10];
      eigRtmp=Eigen::Quaterniond(eigRtmp).normalized().toRotationMatrix();
      if (i==0){
	mTbc=cv::Mat::eye(4,4,CV_32F);
	for (int j=0;j<3;++j){
	  mTbc.at<float>(j,3)=fnT[i][j*4+3];
	  for (int k=0;k<3;++k) mTbc.at<float>(j,k)=eigRtmp(j,k);
	}
      }else{
	mTbo=cv::Mat::eye(4,4,CV_32F);
	for (int j=0;j<3;++j){
	  mTbo.at<float>(j,3)=fnT[i][j*4+3];
	  for (int k=0;k<3;++k) mTbo.at<float>(j,k)=eigRtmp(j,k);
	}
      }
    }//cout<<mTbc<<endl<<mTbo<<endl;
    //load rc,vscale
    double rc=fSettings["Encoder.rc"];EncData::Setvscale(fSettings["Encoder.scale"]);
    //load Sigma etad & etawi
    cv::FileNode fnSig[2]={fSettings["Encoder.Sigmad"],fSettings["IMU.SigmaI"]};
    cv::Mat Sigma1(2,2,CV_32F),Sigma2(3,3,CV_32F);
    for (int i=0;i<2;++i) for (int j=0;j<2;++j) Sigma1.at<float>(i,j)=fnSig[0][i*2+j];
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) Sigma2.at<float>(i,j)=fnSig[1][i*3+j];
    OdomPreIntegrator::SetParam(rc,Sigma1,Sigma2);
    //load delay
    mDelayOdom=-(double)fSettings["Camera.delaytoimu"];

    //old ORBSLAM2 code
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mtmGrabDelay=chrono::steady_clock::now();
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::CacheOdom(const double &timestamp, const double* odomdata, const char mode){//different thread from GrabImageX
  //you can add some odometry here for fast Tcw retrieve(e.g. 200Hz)
  unique_lock<mutex> lock(mMutexOdom);
  switch (mode){
    case System::ENCODER://only encoder
      if (mlOdomEnc.empty()){
	mlOdomEnc.push_back(EncData(odomdata,timestamp));
	miterLastEnc=mlOdomEnc.begin();
      }else
	mlOdomEnc.push_back(EncData(odomdata,timestamp));
      break;
    case System::IMU://only qIMU
      if (mlOdomIMU.empty()){
	mlOdomIMU.push_back(IMUData(odomdata,timestamp,0));
	miterLastIMU=mlOdomIMU.begin();
      }else
	mlOdomIMU.push_back(IMUData(odomdata,timestamp,0));
      break;
    case System::BOTH://both encoder & qIMU
      if (mlOdomEnc.empty()){
	mlOdomEnc.push_back(EncData(odomdata,timestamp));
	miterLastEnc=mlOdomEnc.begin();
      }else
	mlOdomEnc.push_back(EncData(odomdata,timestamp));
      if (mlOdomIMU.empty()){
	mlOdomIMU.push_back(IMUData(odomdata+2,timestamp,0));
	miterLastIMU=mlOdomIMU.begin();
      }else
	mlOdomIMU.push_back(IMUData(odomdata+2,timestamp,0));
      break;
    case System::RESERVEDODOM://only IMU
      if (mlOdomIMU.empty()){
	mlOdomIMU.push_back(IMUData(odomdata,timestamp,1));
	miterLastIMU=mlOdomIMU.begin();
      }else
	mlOdomIMU.push_back(IMUData(odomdata,timestamp,1));
      break;
  }
  
  return cv::Mat();
}
cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mtmGrabDelay=chrono::steady_clock::now();
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    //may be improved here!!!
    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else{
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
	}
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
	else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);//here extracting the ORB features of ImGray
    
    cv::Mat img[2]={imRGB.clone(),imD.clone()};
    Track(img);

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mtmGrabDelay=chrono::steady_clock::now();
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::PreIntegration(const char type){
  unique_lock<mutex> lock(mMutexOdom);
  switch (type){//0/2 will cull 2 Odom lists,1 will shift the pointer
    case 0://for 0th keyframe/frame: erase all the data whose tm<=mCurrentFrame.mTimeStamp but keep the last one
      if (!mlOdomEnc.empty()){
	list<EncData>::iterator iter=mlOdomEnc.end();
	while (iter!=mlOdomEnc.begin()){
	  if ((--iter)->tm<=mCurrentFrame.mTimeStamp) break;//get last one
	}
	if (iter!=mlOdomEnc.end()&&iter->tm<=mCurrentFrame.mTimeStamp-1./mMaxFrames) ++iter;//time error allow 33ms(1 pic time of 30Hz)
	mlOdomEnc.erase(mlOdomEnc.begin(),iter);//retain the last EncData used to calculate the Enc PreIntegration
	miterLastEnc=mlOdomEnc.begin();
      }
      if (!mlOdomIMU.empty()){
	list<IMUData>::iterator iter=mlOdomIMU.end(),iter2;char type=-1;
	while (iter!=mlOdomIMU.begin()){
	  if ((--iter)->tm<=mCurrentFrame.mTimeStamp) break;
	}
	iter2=iter;//last one
	IMUData imudataTmp(nullptr,0,-1);
	if (iter!=mlOdomIMU.end()) ++iter;
	while (iter!=mlOdomIMU.begin()){
	    if ((--iter)->type!=1&&iter->tm>mCurrentFrame.mTimeStamp-1./mMaxFrames){
	      imudataTmp=*iter;
	      break;
	    }
	}
	mlOdomIMU.erase(mlOdomIMU.begin(),iter2);
	if (imudataTmp.type!=-1) mlOdomIMU.push_front(imudataTmp);//retain the last IMUData for calculation of qIMU PreIntegration
	miterLastIMU=mlOdomIMU.begin();
      }
      break;
    case 1:{
      //PreIntegration between 2 frames
      double tmSyncOdom=mLastFrame.mTimeStamp;//Integral Interval (tmSyncOdom,mCurrentFrame.mTimeStamp]
      if (!mlOdomEnc.empty()){
	for (;miterLastEnc!=mlOdomEnc.end();++miterLastEnc){//search a data whose tm has min|mtmSyncOdom-tm| s.t. tm<=mtmSyncOdom
	  if (miterLastEnc->tm>tmSyncOdom){
	    if (miterLastEnc!=mlOdomEnc.begin()) --miterLastEnc;
	    break; 
	  }
	}
	list<EncData>::iterator iter=mlOdomEnc.end();
	while (iter!=miterLastEnc){//the begin/iteri for this frame, find the pback/iterj for this frame
	  if ((--iter)->tm<=mCurrentFrame.mTimeStamp) break;//get last one
	}
	if (iter!=mlOdomEnc.end()){
	  if (iter->tm<=mCurrentFrame.mTimeStamp-1./mMaxFrames) ++iter;//time error allow 33ms(1 pic time of 30Hz)
	  else if (miterLastEnc->tm<=tmSyncOdom&&miterLastEnc->tm>tmSyncOdom-1./mMaxFrames){//Enci exits then delta~xij(phi,p) can be calculated
	      mCurrentFrame.SetPreIntegrationList<EncData>(miterLastEnc,iter);
	  }
	}
	if (iter!=mlOdomEnc.end())
	  miterLastEnc=iter;//update miterLastEnc pointing to the last one of this frame/begin for next frame
	else
	  miterLastEnc=--iter;//if not exit please don't point to the end()! so we have to check the full restriction of miterLastX
      }
      if (!mlOdomIMU.empty()){
	for (list<IMUData>::iterator iteri=miterLastIMU;iteri!=mlOdomIMU.end();++iteri){//search a data whose tm has min|mtmSyncOdom-tm| s.t. tm<=mtmSyncOdom & type!=1
	  if (iteri->tm>tmSyncOdom){
	    break; 
	  }else if (iteri->type!=1) miterLastIMU=iteri;
	}
	list<IMUData>::iterator iter=mlOdomIMU.end(),iter2;
	while (iter!=miterLastIMU){
	  if ((--iter)->tm<=mCurrentFrame.mTimeStamp) break;
	}
	list<IMUData>::iterator iterj=iter;//last one with any type
	if (iter!=mlOdomIMU.end()) ++iter;
	while (iter!=miterLastIMU){
	    if ((--iter)->type!=1&&iter->tm>mCurrentFrame.mTimeStamp-1./mMaxFrames){
	      iterj=iter;//last one with type!=1
	      if (miterLastIMU->tm<=tmSyncOdom&&miterLastIMU->type!=1&&miterLastIMU->tm>tmSyncOdom-1./mMaxFrames){//qiIMU exists then delta~Rij.t() can be calculated
		mCurrentFrame.SetPreIntegrationList<IMUData>(miterLastIMU,iter);
	      }
	      break;
	    }
	}
	if (iterj!=mlOdomIMU.end())
	  miterLastIMU=iterj;//update miterLastIMU pointing to the last one of this frame with type!=1 if exits else with any type
	else
	  miterLastIMU=--iterj;//if not exit please don't point to the end()!
      }
      mCurrentFrame.PreIntegration(&mLastFrame);
      break;}
    case 2:
      //PreIntegration between 2 KFs & cull 2 odom lists: erase all the data whose tm<=mpReferenceKF->mTimeStamp but keep the last one
      if (!mlOdomEnc.empty()){
	list<EncData>::iterator iter=mlOdomEnc.end();
	while (iter!=mlOdomEnc.begin()){
	  if ((--iter)->tm<=mpReferenceKF->mTimeStamp) break;//get last one
	}
	if (iter!=mlOdomEnc.end()){
	  if (iter->tm<=mpReferenceKF->mTimeStamp-1./mMaxFrames) ++iter;//time error allow 33ms(1 pic time of 30Hz)
	  else{
	    list<EncData>::iterator iteri=mlOdomEnc.begin();
	    if (iteri->tm<=mpLastKeyFrame->mTimeStamp){//Enci exits then delta~xij(phi,p) can be calculated, here only use half judgement for if iteri->tm <=mtmSyncOdom, it must >mtmSyncOdom-1./mMaxFrames for type==0/1's process(only keep the last acceptable data)
	      mpReferenceKF->SetPreIntegrationList<EncData>(iteri,iter);
	    }
	  }
	}
	mlOdomEnc.erase(mlOdomEnc.begin(),iter);//retain the last EncData used to calculate the Enc PreIntegration
	miterLastEnc=mlOdomEnc.begin();//maybe end() but we handle it in the CacheOdom()
      }
      if (!mlOdomIMU.empty()){
	list<IMUData>::iterator iter=mlOdomIMU.end(),iter2;
	while (iter!=mlOdomIMU.begin()){
	  if ((--iter)->tm<=mpReferenceKF->mTimeStamp) break;
	}
	iter2=iter;//last one
	IMUData imudataTmp;
	if (iter!=mlOdomIMU.end()) ++iter;
	while (iter!=mlOdomIMU.begin()){
	    if ((--iter)->type!=1&&iter->tm>mpReferenceKF->mTimeStamp-1./mMaxFrames){
	      imudataTmp=*iter;
	      list<IMUData>::iterator iteri=mlOdomIMU.begin();
	      if (iteri->tm<=mpLastKeyFrame->mTimeStamp){//qiIMU exists then delta~Rij.t() can be calculated, only need half judgement
		mpReferenceKF->SetPreIntegrationList<IMUData>(iteri,iter);
	      }
	      break;
	    }
	}
	mlOdomIMU.erase(mlOdomIMU.begin(),iter2);
	if (imudataTmp.type!=-1) mlOdomIMU.push_front(imudataTmp);//retain the last IMUData for calculation of qIMU PreIntegration
	miterLastIMU=mlOdomIMU.begin();
      }
      mpReferenceKF->PreIntegration(mpLastKeyFrame);//mpLastKeyFrame cannot be bad here for mpReferenceKF hasn't been inserted (SetBadFlag only for before KFs)
      break;
  }
}

void Tracking::Track(cv::Mat img[2])
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
    
    //delay control
    {
    unique_lock<mutex> lock2(mMutexOdom);
    if (!mlOdomEnc.empty()&&mlOdomEnc.back().tm>=mCurrentFrame.mTimeStamp&&
      !mlOdomIMU.empty()&&mlOdomIMU.back().tm>=mCurrentFrame.mTimeStamp){//2 lists data is enough for deltax~ij
    }else{//then delay some ms to ensure some Odom data to come if it runs well
      lock2.unlock();
      mtmGrabDelay+=chrono::duration_cast<chrono::nanoseconds>(chrono::duration<double>(mDelayOdom));//delay default=20ms
      while (chrono::steady_clock::now()<mtmGrabDelay) sleep(0.001);//allow 1ms delay error
    }//if still no Odom data comes, deltax~ij will be set unknown
    }

    if(mState==NOT_INITIALIZED)
    {
	PreIntegration();//PreIntegration Intialize
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization(img);
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

	PreIntegration(1);
        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK||mState==ODOMOK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();//so use the replaced ones

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)//if last frame relocalizes, there's no motion could be calculated
                {
                    bOK = TrackReferenceKeyFrame();//match with rKF, use lF as initial & m-o BA
		    cout<<fixed<<setprecision(6);
		    cout<<red"TrackRefKF()"white<<" "<<mCurrentFrame.mTimeStamp<<" "<<mCurrentFrame.mnId<<" "<<(int)bOK<<endl;
                }
                else
                {
                    bOK = TrackWithMotionModel();//match with lF, use v*lF(velocityMM) as initial & m-o BA
                    if(!bOK){
                        bOK = TrackReferenceKeyFrame();
			cout<<red"TrackRefKF()2"white<<" "<<mCurrentFrame.mTimeStamp<<" "<<mCurrentFrame.mnId<<" "<<(int)bOK<<endl;
		    }
                }
            }
            else
            {
                bOK = Relocalization();
		cout<<green"Relocalization()"white<<" "<<mCurrentFrame.mTimeStamp<<" "<<mCurrentFrame.mnId<<" "<<(int)bOK<<endl;
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();//++mnFound in this map point
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;//firstly use last mState==OK Frame.mpReferenceKF, maybe use most covisible KF as the (mCurrentFrame.)mpReferenceKF in TrackLocalMap()

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK){
                bOK = TrackLocalMap();
		if (!bOK)
		  cout<<red"TrackLocalMap() failed!"white<<endl;
	    }
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else{//we shouldn't make it LOST for robustness
            //mState=LOST;
	    //use Odom data to get mCurrentFrame.mTcw
	    if (!mLastTwcOdom.empty()){//when odom data comes we suppose it cannot be empty() again
	      {
		if (mtimestampOdom>mLastTimestamp){
		  mVelocity=mTcwOdom*mLastTwcOdom;//try directly using odom result, Tc2c1
		  mVtmspan=mtimestampOdom-mLastTimestamp;//update deltat
		}else{//==0 then keep the old mVelocity and deltat
		}
	      }
	      if (!mVelocity.empty()){
		cout<<green<<mVelocity.at<float>(0,3)<<" "<<mVelocity.at<float>(1,3)<<" "<<mVelocity.at<float>(2,3)<<white<<endl;
		
		if (mVtmspan==0){
		  mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);//Tc2c1*Tc1w, !mLastFrame.mTcw.empty() must be true
		  cout<<red<<"mVtmspan == 0!"<<white<<endl;
		}else{
		  Eigen::AngleAxisd angAxis(Converter::toMatrix3d(mVelocity.rowRange(0,3).colRange(0,3)));
		  Eigen::AngleAxisd angAxisToUse(angAxis.angle()*(mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/mVtmspan,angAxis.axis());
		  Eigen::Vector3d trans(mVelocity.at<float>(0,3),mVelocity.at<float>(1,3),mVelocity.at<float>(2,3));
		  mCurrentFrame.SetPose(Converter::toCvSE3(angAxisToUse.matrix(),trans*(mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/mVtmspan)
		  *mLastFrame.mTcw);
		}
		//it's difficult to get mCurrentFrame.mvbOutlier like motion-only BA
		mState=ODOMOK;
		cout<<"ODOM KF: "<<mCurrentFrame.mnId<<endl;
	      }else{
		mState=LOST;
		cout<<red"Error when mVelocity.empty()"<<white<<endl;
	      }
	    }else mState=LOST;
	}

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;//Tc2c1/Tcl
                //mVtmspan=mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp;
            }
            else{
                mVelocity = cv::Mat();//can use odometry data here!
		cout<<red"Error in mVelocity=cv::Mat()"<<white<<endl;
	    }

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)//if the mappoint of th i feature is added in to the pMap
                    if(pMP->Observations()<1)//if now no KF observes it, delete it from the vmap in Frame
                    {
                        mCurrentFrame.mvbOutlier[i] = false;//though motion-only BA will initialize this, here is for new MapPoints created by CreateNewKeyFrame(), they must be inliers
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints, for they're only used for current Tcw calculation
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe!!
            if(NeedNewKeyFrame()){
                CreateNewKeyFrame(img);//only create the only CurrentFrame viewed MapPoints without inliers+outliers in mpMap, to avoid possibly replicated MapPoints
		/*unique_lock<std::mutex> lock(mpSystem->mMutexPose);
		static double stlastUpdateSysTOdomStamp=-1;
		if (!mpSystem->mTcwOdom.empty()&&stlastUpdateSysTOdomStamp+3<mCurrentFrame.mTimeStamp){//allow 3s drift
		  mpSystem->mTcwOdom=mCurrentFrame.mTcw;
		  mpSystem->mTwcOdom=mCurrentFrame.mTcw.inv();
		  mpSystem->mtimestampOdom=mCurrentFrame.mTimeStamp;
		  stlastUpdateSysTOdomStamp=mCurrentFrame.mTimeStamp;
		}*/
	    }

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {//new created MPs' mvbOutlier[j] is default false
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])//delete the final still outliers mappoints in Frame
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }else if (mState==ODOMOK){//if it's lost in Camera mode we use Odom mode
	    // not necessary to update motion model for mVelocity is already got through odom data
	    
	    mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
	    if(NeedNewKeyFrame()){
                CreateNewKeyFrame(img,mState);//only create the only CurrentFrame viewed MapPoints without inliers+outliers in mpMap, to avoid possibly replicated MapPoints
	    }
	    for(int i=0; i<mCurrentFrame.N;i++)
            {//new created MPs' mvbOutlier[j] is default false
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])//delete the final still outliers mappoints in Frame
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
	}

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)//useless here?need test
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);//copy constructor for some Mat.clone() and vec<>[][] deep copy
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();//when it's lost ,these may be useless?
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);//false if it isn't lost, when it has Tcw, it stll can be LOST for not enough inlier MapPoints
	
	if (!mTcwOdom.empty()){
	  mLastTwcOdom=mTwcOdom.clone();
	  mLastTimestamp=mtimestampOdom;
	}
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);//it's key value to judge
    }

}


void Tracking::StereoInitialization(cv::Mat img[2])
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
	pKFini->Img[0]=img[0];pKFini->Img[1]=img[1];

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
	//every points with depth info!
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);//use camera intrinsics to backproject the piont from (u,v) to x3Dw
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);//add which KF with the order of features in it has observed this mappoint
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();//choose the observed descriptor having the least median distance to the left
                pNewMP->UpdateNormalAndDepth();//calc the mean viewing direction and max/min dist?
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;//the realization of the AddMapPiont in class Frame
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);//add it to the local mapper KF list

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;//if it's replaced by localMapper, use the replaced one
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);//stricter then SBP in TrackLocalMap()
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);//match with rKF,rectify the vpMapPointMatches, SBBoW better than SBP for the mCurrentFrame.Tcw is unknown

    if(nmatches<15)//looser than 20 in TrackWithMotionModel()
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;//use temporary vector<MapPoint*> for not believe SBBow() so much
    mCurrentFrame.SetPose(mLastFrame.mTcw);//but use lF as the initial value for BA

    Optimizer::PoseOptimization(&mCurrentFrame);//motion-only BA

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];//use temporary pointer to avoid mutex problem?

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;//useless here
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)//where EraseObservation()/SetBadFlag()?
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;//Track ok when enough inlier MapPoints
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    //similar with the part in CreateNewKeyFrame()
    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);//different here

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;//can be optimzed here
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);//here 0.9 is useless

    // Update last frame pose according to its reference keyframe for rKF may be rectified
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);//Tc2c1*Tc1w

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);//has CurrentFrame.mvpMapPoints[bestIdx2]=pMP; in this func. then it can use m-o BA

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));//it's important for SBP() will not rectify the alreay nice CurretFrame.mvpMapPoints
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);//motion-only BA

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;//change to VO mode if the inlier MapPoint is too few i.e. robot goes to the new environment outside of the given map
        return nmatches>20;//Track ok when enough inlier matches
    }

    return nmatchesMap>=10;//Track ok when enough inlier MapPoints
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);//motion-only BA, for added matching MP&&KeyPoints in SearchLocalPoints();
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)//why not include System::RGBD?
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently (recent 1s)
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)//notice it's a class data member
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)//the settings fps used here, if at initial step add new KF quickly while at relocalisation step add it slowly
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)//just check for one(with ur>=0) KF demand for RGBD
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);//the number of good MinObs(for Monocular) KFs tracked MapPoints in the RefKF

    // check if Local Mapping accept keyframes or LM thread is idle
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)//it's a close point
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])//it's a inlier map point or tracked one
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);//τt=100(enough dis)( τc=70(enough info) for stereo/RGBD to insert a new KF

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)//is it necessary for this stricter enough distance threshold?
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;//time span too long(1s)
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);//for minF=0, if LocalMapper is idle
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;//rgbd/stereo tracking weak outside(large part are far points)
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);//not too close && not too far
    
    //Condition 3: odom && time conditon && min new close points' demand
    const bool c3=(mState==ODOMOK)&&(c1a||c1b||c1c)&&(nNonTrackedClose>70);

    if((c1a||c1b||c1c)&&c2||c3)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else//it wating queue is too long still refuse to insert new KFs
                    return false;
            }//it it's monocular, LocalMapper not idle -> refuse to insert new KFs
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame(cv::Mat img[2],eTrackingState state)
{
    if(!mpLocalMapper->SetNotStop(true))//if localMapper is stopped by loop closing thread/GUI, cannot add KFs; during adding process, it cannot be stopped by others
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB,state);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;
    
    PreIntegration(2);//zzh

    if(mSensor!=System::MONOCULAR)
    {	
	pKF->Img[0]=img[0];pKF->Img[1]=img[1];
	
	
        mCurrentFrame.UpdatePoseMatrices();//UnprojectStereo() use mRwc,mOw

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest close points
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);//is there memory leak?
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap,state);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF,state);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched (in TrackWithMotionModel()/...), \
    all of (these) map points created by CreateNewKeyFrame()/StereoInitialization()/{UpdateLastFrame()in Localization mode/\
    CreateNewMapPoints() in LocalMapping}
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;//don't need to match it in SBP()
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)//jump the already in-mCurrentFrame.mvpMapPoints MapPoints
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching,like mbTrackInView=true...)
	//judge if mCurrentFrame's centre is in the effective descriptor area(scale&&rotation invariance) of the MapPoint(with best descriptor&&normalVector)
        if(mCurrentFrame.isInFrustum(pMP,0.5))//no problem for mRcw,mtcw for mCurrentFrame.SetPose() in TrackWithMotionModel()/TrackReferenceKeyFrame()
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);//0.8 is the threshold for mindist/mindist2(<th is nice matching)
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);//rectify the mCurrentFrame.mvpMapPoints
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization,but visualized MapPoints are the last F ones
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)//current F visible MapPoints initial mnTrackReferenceForFrame==0(mCurrentFrame.mnId entering this func. cannot be 0)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;//so it's for avoiding redundant addition
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);//looser than covisibility graph demand
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;//to avoid repetition when selecting neighbor KFs(2nd layer covisible KFs&& neighbors of the spanning tree)
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)//avoid for replicated push_back for different itKF, this cannot be mCurrentFrame(not KF now)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)//still maybe rectify this two refKF variables in CreateNewKeyFrame()
    {
        mpReferenceKF = pKFmax;//highest/max covisible weight/MPs KF
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);//similar threshold in TrackReferenceKeyFrame()

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)//same threshold in TrackReferenceKeyFrame()
            {
                vbDiscarded[i] = true;
                continue;//useless
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);//get transformation by 3D-2D matches
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reaches max. iterations discard keyframe
            if(bNoMore)//to avoid too long time in RANSAC, at most 300 iterations from while start here
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();//vvpMapPointMatches[i].size()/mCurrentFrame.mvpMapPoints.size()

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);//use RANSAC P4P to select good inlier 3D-2D matches, then use all these matches to motion-BA

                if(nGood<10)//without checking pMP->Observation(), the number of inliers by motion-BA, the same threshold as TrackWithMotionModel()/TrackReferenceKeyFrame()
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)//delete outliers
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse or fine window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);//using additional matches to motion-BA

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])//include outliers of just former BA for nGood isn't changed or don't believe former wide SBP() BA
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }//why don't delete mCurrentFrame.mvbOutlier when nGood>=50?
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)//the same threshold as TrackLocalMap() in Relocalization mode
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;
    
    //for monocular!
    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM
