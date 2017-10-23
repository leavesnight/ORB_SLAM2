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



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

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


namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false),
        mtimestampOdom(-1)
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
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
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
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
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
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
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

void System::SaveMap(const string &filename){
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

#include<sys/stat.h>
#include<sys/types.h>

int System::mkdir_p(string foldername,int mode){
  if (foldername.empty()) return -1;
  if (mkdir(foldername.c_str(),mode)==-1){
	int pos=string::npos;
	if (foldername[foldername.length()-1]=='/') pos=foldername.length()-2;
	if (mkdir_p(foldername.substr(0,foldername.rfind('/',pos)),mode)==-1) return -1;
	else return mkdir(foldername.c_str(),mode);
  }else return 0;
}
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

bool System::TrackOdom(const double &timestamp, const double* odomdata, const char mode){
//   cout<<blue<<"OdomData: ";
//   int nTotalNum=2+4+3*3;
//   for (int i=0;i<nTotalNum;++i)
//     cout<<odomdata[i]<<" ";
//   cout<<white<<endl;
  
  static const double xbase_c[3]={0.293+0.225,-0.015,0.578-0.105};//from base frame to camera frame(ROS)/from camera coordinate to base coordinate(ORBSLAM)
  static Eigen::Isometry3d Tbase_c(Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d(1,0,0)));//Roll/R23
  
  static const int ppr=400,datatime=10;//pulse per revolution,the span of the hall sensor data
  static const double wheelradius=0.105,carradius=0.280,vscaleforhall=2.0/ppr*M_PI*wheelradius/datatime;//radius for the driving wheels(m),the half distance between two driving wheels
  static const double wscaleforhall=vscaleforhall/carradius;
  static const double xo_base[3]={0,0,0};//the translational difference from the centre of two driving wheels to base_link frame(the same)
  static const double xoi_o[3]={0.024,0.324,-0.461};
  static int estimate_mode=0;
  static double st_vtmpt,st_wtmpt;//lasttime is replaced by mtimestampOdom
  double vtmpt,wtmpt;
  double deltat=0;
  double vtmp,wtmp,v[2]={odomdata[0],odomdata[1]},arrq[4]={odomdata[2],odomdata[3],odomdata[4],odomdata[5]};
  static double st_xt[3]={0};//x y theta
  double xt[3];//temporary pose
  
  Eigen::Isometry3d To_base(Eigen::Isometry3d::Identity()),Tbase_o;
  static Eigen::Isometry3d Toi_o(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(0,0,1)));
  static Eigen::Isometry3d To0_wi(Eigen::Isometry3d::Identity());//transformation/change from  the first odometry frame(centre of two wheels) to IMU internal frame(ros version meaning,not slam or coordinate meaning)
  To_base.pretranslate(Eigen::Vector3d(xo_base));Tbase_o=To_base.inverse();
  
  //get the Tw_base/Todom_baselink
  if (mtimestampOdom<0){
    //xt[2]=xt[1]=xt[0]=0;
    if (mode==1){//if you may use mode==1, this must be true at first time
      //get initial Tbase_c
      Tbase_c.prerotate(Eigen::AngleAxisd(-M_PI/2,Eigen::Vector3d(0,0,1)));//Y/R12,notice the order R12*R23=R13/YPR!
      Tbase_c.pretranslate(Eigen::Vector3d(xbase_c));
      
      Toi_o.prerotate(Eigen::AngleAxisd(M_PI/2,Eigen::Vector3d(1,0,0)));
      //cout<<Toi_o.rotation().eulerAngles(2,1,0).transpose()<<endl;
      Toi_o.pretranslate(Eigen::Vector3d(xoi_o));
      
      To0_wi.prerotate(Eigen::Quaterniond(arrq));//actually use quaterniond.inverse() has some numerical error, u'd better use conjugate()!
      To0_wi=(To0_wi*Toi_o).inverse();//Tw_wi=(Twi_oi*Toi_o)^(-1);
      //std::cout<<arrq[0]<<" "<<arrq[1]<<" "<<arrq[2]<<" "<<arrq[3]<<std::endl;
      //To0_wi=To0_wi.inverse();//Toi0_wi
    }
    vtmpt=(v[0]+v[1])/2*vscaleforhall;//v1=v-vw;v2=v+vw => v=(v1+v2)/2
    wtmpt=(v[1]-v[0])/2*wscaleforhall;// => w=(v2-v1)/2/r
  }else{
    double theta_tmp;
    if (mode==1){
      Eigen::Isometry3d To0_o(Toi_o);//suppose w means o0 not cr0
      To0_o.prerotate(Eigen::Quaterniond(arrq));//Tw_o=Tw_oi*Toi_o=Tw_wi*Twi_oi*Toi_o;here it's just Twi_o
      To0_o=To0_wi*To0_o;//Tw_o=Tw_wi*Twi_o;end suppose
      //Eigen::Isometry3d To0_o(Eigen::Isometry3d::Identity()),Twi_oi(Eigen::Quaterniond(arrq).toRotationMatrix());
      //To0_o=To0_wi*Twi_oi;
      Eigen::AngleAxisd rotatvec_tmp(To0_o.rotation());
      theta_tmp=(rotatvec_tmp.angle()*rotatvec_tmp.axis()).dot(Eigen::Vector3d(0,0,1));
      if (theta_tmp>M_PI) theta_tmp-=2*M_PI;
      else if (theta_tmp<=-M_PI) theta_tmp+=2*M_PI;
      //double theta2=To0_o.rotation().eulerAngles(2,1,0)[0];
      streamsize nTmp=cout.precision(3);
      //std::cout<<theta_tmp*180/M_PI<<" degrees"<<std::endl<<setprecision(nTmp);
    }
    ///hall sensor data(counts during 10s)/400*pi*2*radius(mm)/1000/10=velocity
    deltat=timestamp-mtimestampOdom;
    vtmpt=(v[0]+v[1])/2*vscaleforhall;wtmpt=(v[1]-v[0])/2*wscaleforhall;
    vtmp=(vtmpt+st_vtmpt)/2;//v1=v-vw;v2=v+vw => v=(v1+v2)/2
    wtmp=(wtmpt+st_wtmpt)/2;// => w=(v2-v1)/2/r
    if (mode==1){
      double delta_rect=theta_tmp-st_xt[2];
      if (delta_rect>M_PI) delta_rect-=2*M_PI;
      else if (delta_rect<=-M_PI) delta_rect+=2*M_PI;
      //Complementary Filter
      const double k_comple=0.008;//it's good between 0.006~0.01
      /*//Kalman Filter/KF
      double vari_rt=vari_wt*deltat*deltat;
      //vari_rt=vari_rt2;var_qt=var_qt2;//use const variances
      double kt_w=(vari_tht+vari_rt)/(vari_tht+var_qt+vari_rt)/deltat;//Kt/deltat*/
      switch (estimate_mode){
	case 0:
	  //Complementary Filter
	  wtmp+=k_comple*delta_rect;
	  break;
	/*case 1:
	  //Kalman Filter/KF
	  if (kt_w>1) kt_w=1;
	  wtmp+=delta_rect*kt_w;
	  vari_tht=(vari_tht+vari_rt)*var_qt/(vari_tht+var_qt+vari_rt);*/
      }
    }
    if (wtmp!=0){
      double v_div_w=vtmp/wtmp,dtheta=wtmp*deltat;
      xt[0]=st_xt[0]+v_div_w*(-sin(st_xt[2])+sin(st_xt[2]+dtheta));//x'=x+-v/w*sinth+v/w*sin(th+w*dt) =v*cos(th)*dt;
      xt[1]=st_xt[1]+v_div_w*(cos(st_xt[2])-cos(st_xt[2]+dtheta));//y'=y+v/w*costh-v/w*cos(th+w*dt) =v*sin(th)*dt;
      xt[2]=st_xt[2]+dtheta;//th(eta)'=th+w*dt;
      if (xt[2]>M_PI) xt[2]-=2*M_PI;
      else if (xt[2]<=-M_PI) xt[2]+=2*M_PI;
    }else{
      xt[0]=st_xt[0]+vtmp*cos(st_xt[2])*deltat;
      xt[1]=st_xt[1]+vtmp*sin(st_xt[2])*deltat;
      xt[2]=st_xt[2];
    }
    //cout<<dtimetmp<<" "<<theta_tmp<<" "<<xt[2]<<" "<<"! "<<kt_w<<" "<<rotatvec_tmp.axis().transpose()<<endl;
  }
  Eigen::Quaterniond q(Eigen::AngleAxisd(xt[2],Eigen::Vector3d(0,0,1)));
  //thie o means the encoder odom frame not the ros odom frame(base_link frame)
  Eigen::Isometry3d Two(Eigen::Isometry3d::Identity()),Tb0_base,Tw_camera;//here world means the SLAM camera frame, not the crystal/base but the first frame of the camera frame
  Two.rotate(q);//here world means the first odom frame
  Two.pretranslate(Eigen::Vector3d(xt[0],xt[1],0));
  Tb0_base=Tbase_o*Two*To_base;//though Tocr.inverse() is independent of the shape of the trajectory!
  Tw_camera=Tbase_c.inverse()*Tb0_base*Tbase_c;//Tcamera0_camera
  
  unique_lock<std::mutex> lock(mMutexPose);
  //first lasttime(mtimestampOdom) initialization must be mode==1
  if (mode==1||mode<2&&mtimestampOdom>=0){//2 just publish, don't update st_xt
    for (int i=0;i<3;++i)
      st_xt[i]=xt[i];
    mtimestampOdom=timestamp;
    st_wtmpt=wtmpt;st_vtmpt=vtmpt;
  }
  //streamsize nTmp=cout.precision(9);
  //cout<<blue<<Tw_camera(0,3)<<" "<<Tw_camera(1,3)<<" "<<Tw_camera(2,3)<<" "<<st_xt[2]*180/M_PI<<setprecision(nTmp)<<white<<endl;
  mTwcOdom=Converter::toCvMat(Tw_camera.matrix());
  mTcwOdom=Converter::toCvMat(Tw_camera.inverse().matrix());
  
  return true;
}

} //namespace ORB_SLAM
