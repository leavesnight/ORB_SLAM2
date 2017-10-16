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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetStereoCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);//mConnectedKeyFrameWeights.erase(pKF) && UpdateBestCovisibles()
    void UpdateConnections();//first connect other KFs to this, then connect this to other KFs/update this->mConnectedKeyFrameWeights...; an undirected graph(covisibility graph)
    void UpdateBestCovisibles();//update mvpOrderedConnectedKeyFrames && mvOrderedWeights by sort()
    std::set<KeyFrame *> GetConnectedKeyFrames();//set made from mConnectedKeyFrameWeights[i].first
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();//mvpOrderedConnectedKeyFrames
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);//get N farthest KFs in covisibility graph(map<KF*,int>)
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);//get some farthest KFs in covisibility graph whose weight<=w
    int GetWeight(KeyFrame* pKF);//mConnectedKeyFrameWeights[pKF](0 no found)

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);//mspChildrens.insert(pKF)
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);//mpParent = pKF,pKF->AddChild(this);
    std::set<KeyFrame*> GetChilds();//mspChildrens
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);//if pKF in mspChildrens

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);//mspLoopEdges.insert(pKF);mbNotErase=true;
    std::set<KeyFrame*> GetLoopEdges();//mspLoopEdges

    // MapPoint observation functions
    void AddMapPoint(MapPoint* pMP, const size_t &idx);//mvpMapPoints[idx]=pMP
    void EraseMapPointMatch(const size_t &idx);//mvpMapPoints[idx]=nullptr
    void EraseMapPointMatch(MapPoint* pMP);//mvpMapPoints[idx corresp. pMP]=nullptr
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    std::set<MapPoint*> GetMapPoints();//make set from good mvpMapPoints
    std::vector<MapPoint*> GetMapPointMatches();//mvpMapPoints
    int TrackedMapPoints(const int &minObs);
    MapPoint* GetMapPoint(const size_t &idx);//mvpMapPoints[idx]

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;//return vec<featureID>, a 2r*2r window search by Grids/Cells speed-up, here no min/maxlevel check unlike Frame.h
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();//mbNotErase=true means it cannot be directly erased by SetBadFlag(), but can use SetErase()
    void SetErase();//try to erase this(&KF) by SetBadFlag() when mbToBeErased==true(SetBadFlag() before)&&mspLoopEdges.empty()

    // Set/check bad flag
    void SetBadFlag();//Erase the relation with this(&KF), Update Spanning Tree&& mbBad+mTcp, erase this(&KF) in mpMap && mpKeyFrameDB
    bool isBad();//mbBad

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:
    //PCL used image
    cv::Mat Img[2];//0 is color,1 is depth

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;//for local Map in tracking
    long unsigned int mnFuseTargetForKF;//for LocalMapping

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;//for local BA in LocalMapping
    long unsigned int mnBAFixedForKF;//for local BA in LocalMapping

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing in GBA
    cv::Mat mTcwGBA;//optimized Tcw in the end of GBA
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;//mpCurrentKF used to correct loop and call GBA

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const std::vector<float> mvuRight; // negative value for monocular points
    const std::vector<float> mvDepth; // negative value for monocular points
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;//used in SaveTrajectoryTUM() in System.cc

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;//covisibility graph need KFs >=15 covisible MapPoints or the KF with Max covisible MapPoints
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;//covisible MPs' number

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    float mHalfBaseline; // Only for visualization

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
