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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class LoopClosing;

class Optimizer
{
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);//add all KFs && MPs(having edges(monocular/stereo) to some KFs) to optimizer, optimize their Pose/Pos and save it in KF.mTcwGBA && MP.mPosGBA
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);//pass all KFs && MPs in pMap to BundleAdjustment(KFs,MPs...)
    
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);//local BA, pKF && its covisible neighbors->SetPose(optimizer,vertex(KFid)), pMPs->SetWorldPos(optimizer.vertex(pMP->mnId+maxKFid+1));\
    (all 1st layer covisibility KFs as rectifying KFs(vertices1), MPs seen in these KFs as rectifying MPs(vertices0),\
    left KFs observing MPs as fixed KFs(vertices1,fixed), connecting edges between MPs && KFs as mono/stereo(KF has >=0 ur) edges, after addition of vertices and edges it still can return by pbStopFlag\
    optimize(5)(can be stopped by pbStopFlag), then if mbAbortBA==false-> optimize only inliers(10), update KFs' Pose && MPs' Pos,normal)
    int static PoseOptimization(Frame* pFrame);//motion-only BA, rectify pFrame->mvbOutlier && pFrame->SetPose(optimizer.vertex(0)), return number of inliers

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                       const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                       const bool &bFixScale);//PoseGraph BA, update all KFs' (in pMap) Pose && all MPs' Pos to optimized one and pLoopKF's Pose is fixed(so id0 KF's Pose may be changed a bit); \
    add new loop edges(including pCurKF-pLoopKF as previous loop edges next time entering this function) && normal far edges(spanning tree edges/previous loop edges/far part of covisibility graph edges); \
    notice there is validation adding LoopConnections as new loop edges and optimization gives more believe on new loop edges if its number is more

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);//relativeMotionS12-only BA(fixed MPs' vertices); \
    rectify vpMatches1[i](erase outliers) && g2oS12(to optimized S12), return the number of inliers; th2=chi2(1-proba_right,2), vpMatches1[i] matched to pKF1->mvpMapPoints[i]
};

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
