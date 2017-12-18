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

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//transformPC
//#include <pcl/common/transforms.h>
//filter
#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/passthrough.h>//use the z direction filed filter
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/eigen.hpp>

#include "Optimizer.h"

//created by zzh over.

#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{
cv::Mat System::TrackOdom(const double &timestamp, const double* odomdata, const char mode){
  cv::Mat Tcw=mpTracker->CacheOdom(timestamp,odomdata,mode);
  
  return Tcw;
}
void System::FullBA(int nIterations,bool bRobust){
  if (mpIMUInitiator->GetSensorIMU())//zzh, full BA
    Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap,mpIMUInitiator->GetGravityVec(),nIterations,NULL,0,bRobust);
  else
    Optimizer::GlobalBundleAdjustment(mpMap,nIterations,NULL,0,bRobust);
}

void System::SaveKeyFrameTrajectoryNavState(const string &filename,bool bUseTbc){
    cout << endl << "Saving keyframe NavState to " << filename << " ..." << endl;
    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;f.open(filename.c_str());f<<fixed;
    
    for(size_t i=0; i<vpKFs.size(); i++){
      KeyFrame* pKF = vpKFs[i];
      // pKF->SetPose(pKF->GetPose()*Two);

      if(pKF->isBad()) continue;

      //For VIO, we should compare the Pose of B/IMU Frame!!! not the Twc but the Twb! with EuRoC's Twb_truth(using Tb_prism/Tbs from vicon0/data.csv) (notice vicon0 means the prism's Pose), and I found state_groundtruth_estimate0 is near Twb_truth but I don't think it's truth!
      if (bUseTbc) pKF->UpdateNavStatePVRFromTcw();//for Monocular
      NavState ns=pKF->GetNavState();
      Eigen::Quaterniond q=ns.mRwb.unit_quaternion();//qwb from Rwb
      Eigen::Vector3d t=ns.mpwb;//twb
      Eigen::Vector3d v=ns.mvwb,bg=ns.mbg+ns.mdbg,ba=ns.mba+ns.mdba;
      f<<setprecision(6)<<pKF->mTimeStamp<<setprecision(7)<<" "<<t(0)<<" "<< t(1)<<" "<<t(2)
      <<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()
      <<" "<<v(0)<<" "<<v(1)<<" "<<v(2)<<" "<<bg(0)<<" "<<bg(1)<<" "<<bg(2)<<" "<<ba(0)<<" "<<ba(1)<<" "<<ba(2)<<endl;
    }
    
    f.close();
    cout << endl << "NavState trajectory saved!" << endl;
}
void System::SaveMap(const string &filename){//maybe can be rewritten in Tracking.cc
  //typedef
  typedef pcl::PointXYZRGB PointT;

  typedef pcl::PointCloud<PointT> PointCloud;
  cout << endl << "Saving keyframe map to " << filename << " ..." << endl;

  vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
  sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin???here it's at the origin
  //cv::Mat Two = vpKFs[0]->GetPoseInverse();

  PointCloud::Ptr pPC(new PointCloud);
  //vector<Eigen::Isometry3d*> poses;
  double fx=fsSettings["Camera.fx"],fy=fsSettings["Camera.fy"],cx=fsSettings["Camera.cx"],cy=fsSettings["Camera.cy"];
  double depthScale=fsSettings["DepthMapFactor"];
  for(size_t i=0; i<vpKFs.size(); i+=2)
  {
      KeyFrame* pKF = vpKFs[i];
      // pKF->SetPose(pKF->GetPose()*Two);
      if(pKF->isBad())
	  continue;

      //cv::Mat R = pKF->GetRotation().t();
      //vector<float> q = Converter::toQuaternion(R);
      //cv::Mat t = pKF->GetCameraCenter();
      //f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
	//<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
      cv::Mat cvTwc=pKF->GetPoseInverse();
      Eigen::Matrix3d r;
      cv::cv2eigen(cvTwc.colRange(0,3).rowRange(0,3),r);
      Eigen::Isometry3d* Twc=new Eigen::Isometry3d(r);
      (*Twc)(0,3)=cvTwc.at<float>(0,3);(*Twc)(1,3)=cvTwc.at<float>(1,3);(*Twc)(2,3)=cvTwc.at<float>(2,3);
      //poses.push_back(Twc);]
      
      PointCloud::Ptr current(new PointCloud);
      cv::Mat color=pKF->Img[0];
      cv::Mat depth=pKF->Img[1];
      Eigen::Isometry3d T=*(Twc);
      for (int v=0;v<color.rows;++v)
	for (int u=0;u<color.cols;++u){
	  unsigned int d=depth.ptr<unsigned short>(v)[u];
	  if (d==0||d>7000) continue;
	  Eigen::Vector3d point;
	  point[2]=d*1.0/depthScale;
	  point[0]=(u-cx)*point[2]/fx;
	  point[1]=(v-cy)*point[2]/fy;
	  Eigen::Vector3d pointWorld=T*point;
	  
	  PointT p;
	  p.x=pointWorld[0];p.y=pointWorld[1];p.z=pointWorld[2];
	  p.b=color.data[v*color.step+u*color.channels()];
	  p.g=color.data[v*color.step+u*color.channels()+1];
	  p.r=color.data[v*color.step+u*color.channels()+2];
	  current->points.push_back(p);
	}
      //depth filter and statistical removal
      /*PointCloud::Ptr pTmp(new PointCloud);
      pcl::StatisticalOutlierRemoval<PointT> statis_filter;//this one costs lots of time!!!
      statis_filter.setMeanK(50);//the number of the nearest points used to calculate the mean neighbor distance
      statis_filter.setStddevMulThresh(1.0);//the standart deviation multiplier,here just use 70% for the normal distri.
      statis_filter.setInputCloud(current);
      statis_filter.filter(*pTmp);
      *pPC+=*pTmp;*/
      *pPC+=*current;
  }
  pPC->is_dense=false;//it contains nan data
  cout<<"PC has "<<pPC->size()<<" points"<<endl;
  
  //voxel filter, to make less volume
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setLeafSize(0.05,0.05,0.05);//1cm^3 resolution;now 5cm
  PointCloud::Ptr pTmp(new PointCloud);
  voxel_filter.setInputCloud(pPC);
  voxel_filter.filter(*pTmp);
  //pTmp->swap(*pPC);
  
  //statistical filter, to eliminate the single points
  pcl::StatisticalOutlierRemoval<PointT> statis_filter;//this one costs lots of time!!!
  statis_filter.setMeanK(50);//the number of the nearest points used to calculate the mean neighbor distance
  statis_filter.setStddevMulThresh(1.0);//the standart deviation multiplier,here just use 70% for the normal distri.
  statis_filter.setInputCloud(pTmp);
  statis_filter.filter(*pPC);
  
  cout<<"after downsampling, it has "<<pPC->size()<<" points"<<endl;
  pcl::io::savePCDFileBinary(filename,*pPC);

  cout << endl << "Map saved!" << endl;
}
#include<sys/stat.h>
#include<sys/types.h>
void System::SaveFrame(string foldername,const cv::Mat& im,const cv::Mat& depthmap,double tm_stamp){
  if (foldername[foldername.length()-1]!='/') foldername+='/';
  string rgbname=foldername+"rgb/",depthname=foldername+"depth/";
  static bool bInit=false;
  if (!bInit){
	if (access(depthname.c_str(),0)==-1){
	  cout<<depthname<<" not exists!"<<endl;
	  if (mkdir_p(depthname,0777)==-1) cout<<"depth mkdir error"<<endl;
	}
	if (access(rgbname.c_str(),0)==-1){
	  cout<<rgbname<<" not exists!"<<endl;
	  if (mkdir_p(rgbname,0777)==-1) cout<<"rgb mkdir error"<<endl;
	}else if (access(depthname.c_str(),0)==0){
	  ofstream fout(foldername+"odometrysensor.txt");
	  fout<<"# odometry data\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp vl vr qx qy qz qw"<<endl;
	  fout.close();
	  fout.open(foldername+"groundtruth.txt");
	  fout<<"# ground truth trajectory\n# file: 'rgbd_dataset_zzh.bag'\n# timestamp tx ty tz qx qy qz qw"<<endl;
	  fout.close();
	  bInit=true;
	}
  }
  char ch[25];//at least 10+1+6+4+1=22
  sprintf(ch,"%.6f.",tm_stamp);//mpTracker->mCurrentFrame.mTimeStamp);
  rgbname=rgbname+ch+"bmp";
  depthname=depthname+ch+"png";

  cv::imwrite(rgbname,im);
  cv::imwrite(depthname,depthmap);
  /*ofstream fout(foldername+"odometrysensor.txt",ios_base::app);
  fout<<fixed;
  fout<<setprecision(6)<<tm_stamp<<" "<<setprecision(3);
  int num_tmp=6;
  for (int i=0;i<num_tmp-1;++i)
    fout<<data[i]<<" ";
  fout<<data[num_tmp-1]<<endl;
  fout.close();
  fout.open(foldername+"groundtruth.txt",ios_base::app);
  fout<<fixed;
  fout<<setprecision(6)<<tm_stamp<<setprecision(4);
  for (int i=0;i<7;++i){
    fout<<" "<<data[2+i];
  }
  fout<<endl;
  fout.close();*/
}
int System::mkdir_p(string foldername,int mode){
  if (foldername.empty()) return -1;
  if (mkdir(foldername.c_str(),mode)==-1){
	int pos=string::npos;
	if (foldername[foldername.length()-1]=='/') pos=foldername.length()-2;
	if (mkdir_p(foldername.substr(0,foldername.rfind('/',pos)),mode)==-1) return -1;
	else return mkdir(foldername.c_str(),mode);
  }else return 0;
}

//created by zzh over.

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{ 
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    //cv::FileStorage 
    fsSettings.open(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad=false;
    if (strVocFile.rfind(".txt")!=string::npos){
      bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
      //mpVocabulary->saveToBinaryFile(strVocFile.substr(0,strVocFile.rfind(".txt"))+".bin");
    }else
      bVocLoad=mpVocabulary->loadFromBinaryFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR,strSettingsFile);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
    
//created by zzh
    //Initialize the IMU Initialization thread and launch
    mpIMUInitiator=new IMUInitialization(mpMap, mSensor==MONOCULAR,strSettingsFile);
    mptIMUInitialization=new thread(&ORB_SLAM2::IMUInitialization::Run,mpIMUInitiator);
    //Set pointers between threads
    mpTracker->SetIMUInitiator(mpIMUInitiator);
    mpLocalMapper->SetIMUInitiator(mpIMUInitiator);
    mpLoopCloser->SetIMUInitiator(mpIMUInitiator);
    mpIMUInitiator->SetLocalMapper(mpLocalMapper);//for Stop LocalMapping thread&&NeedNewKeyFrame() in Tracking thread
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }
    
    //important tracking function!
    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpIMUInitiator->SetFinishRequest(true);//zzh
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA() || !mpIMUInitiator->GetFinish()){//changed by zzh
      usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin???here it's at the origin
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
