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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);//unused here

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,//used by FrameDrawer, not Tracking
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3,
        ODOMOK=4
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;//ORB_SLAM2::System::eSensor

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;//used by FrameDrawer

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;//true for lost!

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track(cv::Mat img[2]=nullptr);//img[2] recorded by KFs

    // Map initialization for stereo and RGB-D
    void StereoInitialization(cv::Mat img[2]=nullptr);

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();//track mCurrentFrame with mpReferenceKF by SBB and motion-only BA(if nmatches is enough), then \
    discard outliers, return nInliers>=10
    void UpdateLastFrame();//update last Frame's Pose by reference KeyFrame&&mlRelativeFramePoses for nonlocalization mode
    bool TrackWithMotionModel();//UpdateLastFrame, use SBP to get mCurrentFrame.mvpMapPoints, then motion-only BA(if nmatches is enough), \
    discard outliers, return nInliers>=10 for nonlocalization mode

    bool Relocalization();

    void UpdateLocalMap();//mpMap->SetReferenceMapPoints(mvpLocalMapPoints), UpdateLocalKeyFrames&&UpdateLocalPoints
    void UpdateLocalPoints();//use mvpLocalKeyFrames[i]->mvpMapPoints to fill mvpLocalMapPoints(avoid duplications by pMP->mnTrackReferenceForFrame)
    void UpdateLocalKeyFrames();//use mCurrentFrame&&its covisible KFs(>=1 covisible MP)&&the KFs' neighbors(10 best covisibility KFs&&parent&&children) \
    to make mvpLocalKeyFrames, update (mCurrentFrame.)mpReferenceKF to max covisible KF

    bool TrackLocalMap();//use UpdateLocalMap&&SearchLocalPoints(mvpLocalMapPoints) to add new matched \
    mvpMapPoints in mCurrentFrame, then motion-only BA to add Pose's accuracy and update mnMatchesInliers&&pMP->mnFound, \
    finally may judge mnMatchesInliers to one ballot veto the mState to make it Lost
    void SearchLocalPoints();//update mCurrentFrame->mvpMapPoints(also discard bad ones)+mvpLocalMapPoints' pMP->mnVisible&&mnLastFrameSeen and \
    call mCurrentFrame.isInFrustum(pMP,0.5), then try to match mvpLocalMapPoints to mCurrentFrame by SBP()(add some mvpMapPoints)

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame(cv::Mat img[2]=nullptr,eTrackingState state=OK);

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;//corresponding to mCurrentFrame(most of time ==mCurrentFrame.mpReferenceKF)
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;//40b, here TUM use 3.2(m)

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;//rectified in TrackLocalMap()

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
    
    //last pose by Odom data
    cv::Mat mLastTwcOdom;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
