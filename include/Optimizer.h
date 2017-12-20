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

#include "Converter.h"//zzh for template
#include "g2otypes.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"//must before linear_solver_cholmod...
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
//#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_cholmod.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "IMUInitialization.h"

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
  template<class KeyFrame>
  int static PoseOptimization(Frame *pFrame, KeyFrame* pLastKF, const cv::Mat& gw,const bool bComputeMarg=false);//2 frames' motion-only BA, automatically fix/unfix lastF/KF and optimize curF/curF&last, if bComputeMarg then save its Hessian
  template<class KeyFrame>
  static void PoseOptimizationAddEdge(KeyFrame* pFrame,vector<g2o::EdgeNavStatePVRPointXYZOnlyPose*> &vpEdgesMono,vector<size_t> &vnIndexEdgeMono,
			       const Matrix3d &Rcb,const Vector3d &tcb,g2o::SparseOptimizer &optimizer,int LastKFPVRId){}//we specialize the Frame version
  void static LocalBAPRVIDP(KeyFrame *pKF, int Nlocal, bool* pbStopFlag, Map* pMap, cv::Mat &gw);
  void static LocalBundleAdjustmentNavStatePRV(KeyFrame* pKF, int Nlocal, bool *pbStopFlag, Map *pMap, cv::Mat gw);//Nlocal>=1(if <1 it's 1)
  void static GlobalBundleAdjustmentNavStatePRV(Map* pMap, const cv::Mat &gw, int nIterations=5, bool *pbStopFlag=NULL,
				    const unsigned long nLoopKF=0, const bool bRobust = true);//add all KFs && MPs(having edges(monocular/stereo) to some KFs) to optimizer, optimize their Pose/Pos and save it in KF.mTcwGBA && MP.mPosGBA
  
  template<class IMUKeyFrameInit>
  static Eigen::Vector3d OptimizeInitialGyroBias(const std::vector<IMUKeyFrameInit*> &vpKFInit,bool bInfo=true);//if bInfo==true, use the R part of SigmaPRV as the Infomation matrix else use Identity(); \
  and if use<Frame> please compute mOdomPreIntIMU before
//   Vector3d static OptimizeInitialGyroBias(const std::list<KeyFrame*> &lLocalKeyFrames);
//   Vector3d static OptimizeInitialGyroBias(const std::vector<KeyFrame*> &vLocalKeyFrames);
//   Vector3d static OptimizeInitialGyroBias(const std::vector<Frame> &vFrames);
//  Vector3d static OptimizeInitialGyroBias(const vector<cv::Mat>& vTwc, const vector<IMUPreintegrator>& vImuPreInt);
  
//created by zzh over.
  
public:
    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);//add all KFs && MPs(having edges(monocular/stereo) to some KFs) to optimizer, optimize their Pose/Pos and save it in KF.mTcwGBA && MP.mPosGBA
    void static GlobalBundleAdjustment(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
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
                                       const bool &bFixScale);//PoseGraph Opt., update all KFs' (in pMap) Pose && all MPs' Pos to optimized one and pLoopKF's Pose is fixed(so id0 KF's Pose may be changed a bit); \
    add new loop edges(including pCurKF-pLoopKF as previous loop edges next time entering this function) && normal far edges(spanning tree edges/previous loop edges/far part of covisibility graph edges); \
    notice there is validation adding LoopConnections as new loop edges and optimization gives more believe on new loop edges if its number is more

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);//relativeMotionS12-only BA(fixed MPs' vertices); \
    rectify vpMatches1[i](erase outliers) && g2oS12(to optimized S12), return the number of inliers; th2=chi2(1-proba_right,2), vpMatches1[i] matched to pKF1->mvpMapPoints[i]
};

//created by zzh
using namespace Eigen;
template <class KeyFrame>
int Optimizer::PoseOptimization(Frame *pFrame, KeyFrame* pLastKF, const cv::Mat& gw, const bool bComputeMarg){
  //automatically judge if fix lastF/KF(always fixed)
  bool bFixedLast=true;
  if (pLastKF->mbPrior) bFixedLast=false;
  // Extrinsics
  Matrix3d Rcb = pFrame->meigRcb;
  Vector3d tcb = pFrame->meigtcb;
  // Gravity vector in world frame
  Vector3d GravityVec = Converter::toVector3d(gw);

  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType * linearSolver;//9*1 is Log(R),t,v/P/pvR, 6*1 bgi,bai/bi/Bias, (3*1 is location of landmark,) 3 types of vertices so using BlockSolverX, though here 9_6 is also OK for unary edge in BA

  //linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
  //sparse Cholesky factorization.
  linearSolver = new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();//linear equation solver changed by JingWang

  g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);//descending/optimization strategy is still LM
  //g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);//try this!
  optimizer.setAlgorithm(solver);
  
  // Set Frame & fixed KeyFrame's vertices, see VIORBSLAM paper (4)~(8)
  const int FramePVRId = 0,FrameBiasId = 1,LastKFPVRId = 2,LastKFBiasId = 3;NavState &nsj=pFrame->mNavState;
  // Set Frame vertex PVR/Bias
  g2o::VertexNavStatePVR * vNSFPVR = new g2o::VertexNavStatePVR();
  vNSFPVR->setEstimate(nsj);
  vNSFPVR->setId(FramePVRId);vNSFPVR->setFixed(false);optimizer.addVertex(vNSFPVR);
  g2o::VertexNavStateBias * vNSFBias = new g2o::VertexNavStateBias();
  vNSFBias->setEstimate(nsj);
  vNSFBias->setId(FrameBiasId);vNSFBias->setFixed(false);optimizer.addVertex(vNSFBias);
  // Set KeyFrame vertex PVR/Bias
  g2o::VertexNavStatePVR * vNSKFPVR = new g2o::VertexNavStatePVR();
  vNSKFPVR->setEstimate(pLastKF->GetNavState());
  vNSKFPVR->setId(LastKFPVRId);vNSKFPVR->setFixed(bFixedLast);optimizer.addVertex(vNSKFPVR);
  g2o::VertexNavStateBias * vNSKFBias = new g2o::VertexNavStateBias();
  vNSKFBias->setEstimate(pLastKF->GetNavState());
  vNSKFBias->setId(LastKFBiasId);vNSKFBias->setFixed(bFixedLast);optimizer.addVertex(vNSKFBias);

  // Set IMU_I/PVR(B) edge(ternary/multi edge) between LastKF-Frame
  const IMUPreintegrator &imupreint=pFrame->mOdomPreIntIMU;
  g2o::EdgeNavStatePVR* eNSPVR = new g2o::EdgeNavStatePVR();
  eNSPVR->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFPVRId)));//PVRi, i is keyframe's id
  eNSPVR->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));//PVRj, j here is frame's id
  eNSPVR->setVertex(2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFBiasId)));//bi
  eNSPVR->setMeasurement(imupreint);//set delta~PVRij/delta~pij,delta~vij,delta~Rij
  eNSPVR->setInformation(imupreint.mSigmaij.inverse());
  eNSPVR->SetParams(GravityVec);
  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;eNSPVR->setRobustKernel(rk);rk->setDelta(sqrt(16.919));//chi2(0.05/0.01,9), 16.919/21.666 for 0.95/0.99 9DoF, but JingWang uses 100*21.666
  optimizer.addEdge(eNSPVR);
  // Set IMU_RW/Bias edge(binary edge) between LastKF-Frame
  g2o::EdgeNavStateBias* eNSBias = new g2o::EdgeNavStateBias();
  eNSBias->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFBiasId)));//bi
  eNSBias->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FrameBiasId)));//bj
  eNSBias->setMeasurement(imupreint);
  Matrix<double,6,6> InvCovBgaRW = Matrix<double,6,6>::Identity();
  InvCovBgaRW.topLeftCorner(3,3)=Matrix3d::Identity()*IMUDataBase::mInvSigmabg2;      	// Gyroscope bias random walk, covariance INVERSE
  InvCovBgaRW.bottomRightCorner(3,3)=Matrix3d::Identity()*IMUDataBase::mInvSigmaba2;   	// Accelerometer bias random walk, covariance INVERSE
  eNSBias->setInformation(InvCovBgaRW/imupreint.mdeltatij);// see Manifold paper (47), notice here is Omega_d/Sigma_d.inverse()
  rk = new g2o::RobustKernelHuber;eNSBias->setRobustKernel(rk);rk->setDelta(sqrt(12.592));//chi2(0.05/0.01,6), 12.592/16.812 for 0.95/0.99 6DoF, but JW uses 16.812
  optimizer.addEdge(eNSBias);
  // Set Prior edge(binary edge) for Last Frame, from mMargCovInv
  g2o::EdgeNavStatePriorPVRBias* eNSPrior=NULL;
  if (!bFixedLast){
    eNSPrior = new g2o::EdgeNavStatePriorPVRBias();
    eNSPrior->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFPVRId)));
    eNSPrior->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(LastKFBiasId)));
    eNSPrior->setMeasurement(pLastKF->mNavStatePrior);eNSPrior->setInformation(pLastKF->mMargCovInv);
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;eNSPrior->setRobustKernel(rk);rk->setDelta(sqrt(25));//thHuberNavState:chi2(0.05,15)=25 or chi2(0.01,15)=30.5779
    optimizer.addEdge(eNSPrior);
  }

  int nInitialCorrespondences=0;

  // Set MapPoint Unary edges/Set MapPoint vertices
  const int N = pFrame->N;//for LastFrame JingWang use Nlast while the VIORBSLAM paper hasn't done this see its Fig.2.! let's try his method!

  vector<g2o::EdgeNavStatePVRPointXYZOnlyPose*> vpEdgesMono;//2*1(_measurement) unary edge<VertexNavStatePVR>
  vector<size_t> vnIndexEdgeMono;
  vpEdgesMono.reserve(N);
  vnIndexEdgeMono.reserve(N);//this can be optimized in RGBD mode
  //for bFixedLast==false
  vector<g2o::EdgeNavStatePVRPointXYZOnlyPose*> vpEdgesMonoLast;
  vector<size_t> vnIndexEdgeMonoLast;

  vector<g2o::EdgeStereoNavStatePVRPointXYZOnlyPose*> vpEdgesStereo;//3*1(ul vl ur) unary edge
  vector<size_t> vnIndexEdgeStereo;
  vpEdgesStereo.reserve(N);
  vnIndexEdgeStereo.reserve(N);

  const float deltaMono = sqrt(5.991);//chi2(0.05,2)
  const float deltaStereo = sqrt(7.815);//chi2 distribution chi2(0.05,3), the huber kernel delta

  {
  unique_lock<mutex> lock(MapPoint::mGlobalMutex);//forbid other threads to rectify pFrame->mvpMapPoints' Position

  for(int i=0; i<N; i++)
  {
      MapPoint* pMP = pFrame->mvpMapPoints[i];
      if(pMP)
      {
	  // Monocular observation
	  if(pFrame->mvuRight[i]<0)//this may happen in RGBD case!
	  {
	      nInitialCorrespondences++;
	      pFrame->mvbOutlier[i] = false;

	      Eigen::Matrix<double,2,1> obs;
	      const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
	      obs << kpUn.pt.x, kpUn.pt.y;

	      g2o::EdgeNavStatePVRPointXYZOnlyPose* e = new g2o::EdgeNavStatePVRPointXYZOnlyPose();

	      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));//here should change to FramePVRId!
	      e->setMeasurement(obs);
	      const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
	      e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

	      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
	      e->setRobustKernel(rk);
	      rk->setDelta(deltaMono);

	      e->SetParams(pFrame->fx,pFrame->fy,pFrame->cx,pFrame->cy,Rcb,tcb,Converter::toVector3d(pMP->GetWorldPos()));

	      optimizer.addEdge(e);

	      vpEdgesMono.push_back(e);
	      vnIndexEdgeMono.push_back(i);
	  }
	  else  // Stereo observation
	  {
	      assert(0&&"Now we just test Mono!but u enter the wrong section!");
	      nInitialCorrespondences++;
	      pFrame->mvbOutlier[i] = false;

	      //SET EDGE
	      Eigen::Matrix<double,3,1> obs;
	      const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
	      const float &kp_ur = pFrame->mvuRight[i];
	      obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

	      //g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();
	      g2o::EdgeStereoNavStatePVRPointXYZOnlyPose* e = new g2o::EdgeStereoNavStatePVRPointXYZOnlyPose();

	      e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(FramePVRId)));//this dynamic_cast is useless
	      e->setMeasurement(obs);//edge parameter/measurement formula output z
	      const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
	      Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;//optimization target block=|e'*Omiga(or Sigma^(-1))*e|, diagonal matrix means independece between x and y pixel noise
	      e->setInformation(Info);//3*3 matrix

	      g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;//optimization target=KernelHuber(block)=H(e)={1/2*e sqrt(e)<=delta;delta(sqrt(e)-1/2*delta) others}
	      e->setRobustKernel(rk);
	      rk->setDelta(deltaStereo);

	      e->SetParams(pFrame->fx,pFrame->fy,pFrame->cx,pFrame->cy,Rcb,tcb,Converter::toVector3d(pMP->GetWorldPos()),&pFrame->mbf);//edge/measurement formula parameter Xw

	      optimizer.addEdge(e);//_error is the edge output

	      vpEdgesStereo.push_back(e);
	      vnIndexEdgeStereo.push_back(i);//record the edge recording feature index
	  }
      }
  }
  //for bFixedLast==false
  if (!bFixedLast) PoseOptimizationAddEdge<KeyFrame>(pLastKF,vpEdgesMonoLast,vnIndexEdgeMonoLast,Rcb,tcb,optimizer,LastKFPVRId);
  }

  if(nInitialCorrespondences<3)//at least P3P（well posed equation） EPnP(n>3) (overdetermined equation)
      return 0;

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
  const float chi2Mono[4]={5.991,5.991,5.991,5.991};
  const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};//chi2(0.05,3), error_block limit(over will be outliers,here also lead to turning point in RobustKernelHuber)
  const int its[4]={10,10,10,10};    

  int nBad=0;int nBadIMU=0;
  for(size_t it=0; it<4; it++)//4 optimizations, each 10 steps, initial value is the same, but inliers are different
  {
      // Reset estimate for vertexj
      vNSFPVR->setEstimate(nsj);vNSFBias->setEstimate(nsj);
      if (!bFixedLast){
	vNSKFPVR->setEstimate(pLastKF->GetNavState());vNSKFBias->setEstimate(pLastKF->GetNavState());
      }
      
      optimizer.initializeOptimization(0);//default edges' level is 0, so initially use all edges to optimize, after it=0, just use inlier edges(_activeEdges) to optimize
      optimizer.optimize(its[it]);//only call _activeEdges[k].computeError()

      nBad=0;
      for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)//for 3D-monocular 2D matches, may entered in RGBD!
      {
	  g2o::EdgeNavStatePVRPointXYZOnlyPose* e = vpEdgesMono[i];

	  const size_t idx = vnIndexEdgeMono[i];

	  if(pFrame->mvbOutlier[idx])
	  {
	      e->computeError();
	  }

	  const float chi2 = e->chi2();

	  if(chi2>chi2Mono[it])
	  {                
	      pFrame->mvbOutlier[idx]=true;
	      e->setLevel(1);
	      nBad++;
	  }
	  else
	  {
	      pFrame->mvbOutlier[idx]=false;
	      e->setLevel(0);
	  }

	  if(it==2)
	      e->setRobustKernel(0);
      }
      for(size_t i=0, iend=vpEdgesMonoLast.size(); i<iend; i++)//for 3D-monocular 2D matches, may entered in RGBD!
      {
	  KeyFrame* pFrame=pLastKF;
	  g2o::EdgeNavStatePVRPointXYZOnlyPose* e = vpEdgesMonoLast[i];

	  const size_t idx = vnIndexEdgeMonoLast[i];

	  if(pFrame->mvbOutlier[idx])
	  {
	      e->computeError();
	  }

	  const float chi2 = e->chi2();

	  if(chi2>chi2Mono[it])
	  {                
	      pFrame->mvbOutlier[idx]=true;
	      e->setLevel(1);
	  }
	  else
	  {
	      pFrame->mvbOutlier[idx]=false;
	      e->setLevel(0);
	  }

	  if(it==2)
	      e->setRobustKernel(0);
      }

      for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)//for 3D-stereo 2D matches
      {
	  g2o::EdgeStereoNavStatePVRPointXYZOnlyPose* e = vpEdgesStereo[i];

	  const size_t idx = vnIndexEdgeStereo[i];

	  if(pFrame->mvbOutlier[idx])//at 1st time, all false for all edges is at level 0 or inliers(supposed),so e._error is computed by g2o
	  {
	      e->computeError();
	  }

	  const float chi2 = e->chi2();//chi2=e'*Omiga*e

	  if(chi2>chi2Stereo[it])
	  {
	      pFrame->mvbOutlier[idx]=true;
	      e->setLevel(1);//adjust the outlier edges' level to 1
	      nBad++;
	  }
	  else
	  {                
	      e->setLevel(0);//maybe adjust the outliers' level to inliers' level
	      pFrame->mvbOutlier[idx]=false;
	  }

	  if(it==2)
	      e->setRobustKernel(0);//let the final(it==3) optimization use no RobustKernel; this function will delete _robustkernel first
      }
      
      if(optimizer.edges().size()<10)//it outliers+inliers(/_edges) number<10 only optimize once with RobustKernelHuber
	  break;
      
/*      if (it<3){
      nBadIMU=0;
      {
	g2o::EdgeNavStatePVR* e=eNSPVR;
	if(e->chi2()>21.666){//if chi2 error too big(5% wrong) or Zc<=0 then outlier
	  e->setLevel(1);++nBadIMU;
	}else e->setLevel(0);
	if (it==2) e->setRobustKernel(0);//cancel RobustKernel
      }
      {
	g2o::EdgeNavStateBias* e=eNSBias;
	if(e->chi2()>16.812){//if chi2 error too big(5% wrong) or Zc<=0 then outlier
	  e->setLevel(1);++nBadIMU;
	}else e->setLevel(0);
	if (it==2) e->setRobustKernel(0);//cancel RobustKernel
      }
      if (eNSPrior!=NULL){
	g2o::EdgeNavStatePriorPVRBias* e=eNSPrior;
	if(e->chi2()>30.5779){//if chi2 error too big(5% wrong) or Zc<=0 then outlier
	  e->setLevel(1);++nBadIMU;
	}else e->setLevel(0);
	if (it==2) e->setRobustKernel(0);//cancel RobustKernel
      }}*/
  }

  // Recover optimized pose and return number of inliers
  g2o::VertexNavStatePVR* vNSPVR_recov = static_cast<g2o::VertexNavStatePVR*>(optimizer.vertex(FramePVRId));
//   cout<<"recovered pwb="<<vNSPVR_recov->estimate().mpwb.transpose()<<" & matches by motion-only BA:"<<nInitialCorrespondences-nBad<<", before Optimized:"<<nInitialCorrespondences<<endl;
  nsj=vNSPVR_recov->estimate();
  g2o::VertexNavStateBias* vNSBias_recov = static_cast<g2o::VertexNavStateBias*>(optimizer.vertex(FrameBiasId));
  const NavState& nsBias_recov = vNSBias_recov->estimate();
  nsj.mdbg=nsBias_recov.mdbg;nsj.mdba=nsBias_recov.mdba;
  pFrame->UpdatePoseFromNS();//update posematrices of pFrame

  // Compute marginalized Hessian H and B, H*x=B, H/B can be used as prior for next optimization in PoseOptimization, dx'Hdx should be small then next optimized result is appropriate for former BA
  if(bComputeMarg){
//     if (nBadIMU>0){}else{
    std::vector<g2o::OptimizableGraph::Vertex*> margVertices;
    margVertices.push_back(optimizer.vertex(FramePVRId));
    margVertices.push_back(optimizer.vertex(FrameBiasId));

    //TODO: how to get the joint marginalized covariance of PVR&Bias, here it's already marginalized for fixed LastKF(for SigmaI ind. with SigmaR then H(0,1)=0,H(1,0)=0)
    g2o::SparseBlockMatrixXd spinv;
    optimizer.computeMarginals(spinv, margVertices);//marginalized by cholmod linearsolver
    // spinv include 2 blocks, 9x9-(0,0) for PVR, 6x6-(1,1) for Bias
    if (bFixedLast){
      Matrix<double,15,15> margCovInv = Matrix<double,15,15>::Zero();
      margCovInv.topLeftCorner(9,9) = spinv.block(0,0)->inverse();//0 corresponding to the FramePVRId & fixed LastKFPVRId's hessianidx=-1
      margCovInv.bottomRightCorner(6,6) = spinv.block(1,1)->inverse();//1 corresponding to the FrameBiasId
      pFrame->mMargCovInv = margCovInv;
    }else{
      Matrix<double,15,15> margCov = Matrix<double,15,15>::Zero();
      margCov.topLeftCorner(9,9) = spinv.block(0,0)->eval();//I think eval() is useless for there's no confusion here/.noalias()=
      margCov.topRightCorner(9,6) = spinv.block(0,1)->eval();
      margCov.bottomLeftCorner(6,9) = spinv.block(1,0)->eval();
      margCov.bottomRightCorner(6,6) = spinv.block(1,1)->eval();
      pFrame->mMargCovInv = margCov.inverse();
    }
    pFrame->mNavStatePrior=nsj;//pLastF->mNavStatePrior is needed for this func. will be called twice and pLastF->mNavState will also be optimized
    pFrame->mbPrior=true;//let next tracking uses unfixed lastF mode!
//     }
  }

  return nInitialCorrespondences-nBad;//number of inliers
}
template <>
void Optimizer::PoseOptimizationAddEdge<Frame>(Frame* pFrame,vector<g2o::EdgeNavStatePVRPointXYZOnlyPose*> &vpEdgesMono,vector<size_t> &vnIndexEdgeMono,
					       const Matrix3d &Rcb,const Vector3d &tcb,g2o::SparseOptimizer &optimizer,int LastFramePVRId);

template <class IMUKeyFrameInit>
Vector3d Optimizer::OptimizeInitialGyroBias(const std::vector<IMUKeyFrameInit*> &vpKFInit, bool bInfo){
  Matrix3d Rcb = Frame::meigRcb;

  // Setup optimizer
  g2o::SparseOptimizer optimizer;
  g2o::BlockSolverX::LinearSolverType * linearSolver;//canbe improved here for using fixed BlockSolver

  linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

  g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

  g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);//suggested by VIORBSLAM paper IV-A
  optimizer.setAlgorithm(solver);

  // Add vertex of gyro bias, to optimizer graph
  g2o::VertexGyrBias * vBiasg = new g2o::VertexGyrBias();
  vBiasg->setEstimate(Vector3d::Zero());//zero bias seed, see IV-A in VIORBSLAM paper
  vBiasg->setId(0);
  optimizer.addVertex(vBiasg);

  // Add unary edges for gyro bias vertex
  int N=vpKFInit.size();
  for(int i=0; i<N; i++){
      if(i==0) continue;// Ignore the first KF
      const IMUPreintegrator& imupreint=vpKFInit[i]->mOdomPreIntIMU;//notice this should be computed before calling this function

      g2o::EdgeGyrBias * eBiasg = new g2o::EdgeGyrBias();
      eBiasg->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
      // measurement is not used in EdgeGyrBias
      eBiasg->deltaRij=imupreint.mRij;//deltaRij/deltaRii+1
      eBiasg->JgRij=imupreint.mJgRij;//Jg_deltaR
      eBiasg->Rwbi=Converter::toMatrix3d(vpKFInit[i-1]->mTcw.rowRange(0,3).colRange(0,3).t())*Rcb;//Rwbi=Rwci*Rcb
      eBiasg->Rwbj=Converter::toMatrix3d(vpKFInit[i]->mTcw.rowRange(0,3).colRange(0,3).t())*Rcb;//Rwbj/Rwbi+1=Rwcj/Rwci+1 * Rcb
      if (bInfo)
	eBiasg->setInformation(imupreint.mSigmaijPRV.block<3,3>(3,3).inverse());
      else
	eBiasg->setInformation(Matrix3d::Identity());//JingWang uses it in vector<Frame>
      optimizer.addEdge(eBiasg);
  }

  // It's actualy a linear estimator, so 1 iteration is enough.
  optimizer.initializeOptimization();
  optimizer.optimize(1);
//   optimizer.setVerbose(true);

  g2o::VertexGyrBias * vBgEst = static_cast<g2o::VertexGyrBias*>(optimizer.vertex(0));

  return vBgEst->estimate();
}

} //namespace ORB_SLAM

#endif // OPTIMIZER_H
