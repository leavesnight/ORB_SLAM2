#include "IMUInitialization.h"
#include "Optimizer.h"

//zzh defined color cout, must after include opencv2
#define red "\033[31m"
#define green "\e[32m"
#define blue "\e[34m"
#define yellow "\e[33m"
#define white "\e[37m"

namespace ORB_SLAM2 {

using namespace std;
using namespace Eigen;

IMUKeyFrameInit::IMUKeyFrameInit(KeyFrame& kf):mTimeStamp(kf.mTimeStamp),mTwc(kf.GetPoseInverse()),mTcw(kf.GetPose()), mpPrevKeyFrame(NULL),//GetPose() already return .clone()
mOdomPreIntIMU(kf.GetIMUPreInt()){//this func. for IMU Initialization cache of KFs, so need deep copy
  mbg_=mba_=Vector3d::Zero();//as stated in IV-A in VIORBSLAM paper 
  const list<IMUData> limu=kf.GetListIMUData();
  mOdomPreIntIMU.SetPreIntegrationList(limu.begin(),--limu.end());
}

cv::Mat IMUInitialization::GetGravityVec(void){
  //unique_lock<mutex> lock(mMutexInitIMU);//now we don't need mutex for it 1stly calculated only in this thread and then it will be a const!
  return mGravityVec;//.clone();//avoid simultaneous operation
}

void IMUInitialization::Run(){
  unsigned long initedid;
  cout<<"start VINSInitThread"<<endl;
  mbFinish=false;
  while(1){
    if(GetSensorIMU()){//at least 4 consecutive KFs, see IV-B/C VIORBSLAM paper
      KeyFrame* pCurKF=GetCurrentKeyFrame();
      if (mdStartTime==-1){ initedid=0;mdStartTime=-2;}
      if(!GetVINSInited() && pCurKF!=NULL && pCurKF->mnId > initedid){
	initedid = pCurKF->mnId;
	if (TryInitVIO()) break;//if succeed in IMU Initialization, this thread will finish, when u want the users' pushing reset button be effective, delete break!
      }
    }
    
    ResetIfRequested();
    if(GetFinishRequest()) break;
    //usleep(3000);
    sleep(3);
  }
  SetFinish(true);
  cout<<"VINSInitThread is Over."<<endl;
}
bool IMUInitialization::TryInitVIO(void){//now it's the version cannot allow the KFs has no inter IMUData in initial 15s!!!
  //at least N>=4
  if(mpMap->KeyFramesInMap()<4){ return false;}//21
  //Recording data in txt files for further analysis
  static bool fopened = false;
  static ofstream fgw,fscale,fbiasa,fcondnum,fbiasg;
  if(mTmpfilepath.length()>0&&!fopened){
    // Need to modify this to correct path
    fbiasg.open(mTmpfilepath+"biasg.txt");//optimized initial bg for these N KFs,3*1
    fgw.open(mTmpfilepath+"gw.txt");//gwafter then gw before,6*1
    fscale.open(mTmpfilepath+"scale.txt");//scale_fine then scale_rough, 2*1 only for Monocular camera//if (mbMonocular) 
    fbiasa.open(mTmpfilepath+"biasa.txt");//optimized initial ba for these N KFs,3*1
    fcondnum.open(mTmpfilepath+"condnum.txt");//for optimized x is 6*1 vector, see (19) in VOIRBSLAM paper, here just show these 6*1 raw data
    if(fbiasg.is_open()&& fgw.is_open() && (fscale.is_open()) && //!mbMonocular||
      fbiasa.is_open()&&fcondnum.is_open())
	fopened = true;
    else{
	cerr<<"file open error in TryInitVIO"<<endl;
	fopened = false;
    }
    fbiasg<<std::fixed<<std::setprecision(6);
    fgw<<std::fixed<<std::setprecision(6);
    fscale<<std::fixed<<std::setprecision(6);
    fbiasa<<std::fixed<<std::setprecision(6);
    fcondnum<<std::fixed<<std::setprecision(6);
  }

  Optimizer::GlobalBundleAdjustment(mpMap, 10);//GBA by only vision 1stly, suggested by JingWang

  // Extrinsics
  cv::Mat Tbc = Frame::mTbc;
  cv::Mat Rbc = Tbc.rowRange(0,3).colRange(0,3);
  cv::Mat pbc = Tbc.rowRange(0,3).col(3);
  cv::Mat Rcb = Rbc.t();
  cv::Mat pcb = -Rcb*pbc;

  // Cache KFs / wait for KeyFrameCulling() over
  while(GetCopyInitKFs()) usleep(1000);
  SetCopyInitKFs(true);//stop KeyFrameCulling() when this copying KFs
  if(mpMap->KeyFramesInMap()<4){ SetCopyInitKFs(false);return false;}//ensure no KeyFrameCulling() during the start of this func. till here

  //see VIORBSLAM paper IV, here N=all KFs in map, not the meaning of local KFs' number
  // Use all KeyFrames in map to compute
  vector<KeyFrame*> vScaleGravityKF = mpMap->GetAllKeyFrames();
  //sort(vScaleGravityKF.begin(),vScaleGravityKF.end(),[](const KeyFrame *a,const KeyFrame *b){return a->mnId<b->mnId;});//we change the set less/compare func. so that we don't need to sort them!
  assert((*vScaleGravityKF.begin())->mnId==0);
  int N = vScaleGravityKF.size();
  KeyFrame* pNewestKF = vScaleGravityKF[N-1];
  // Store initialization-required KeyFrame data
  vector<IMUKeyFrameInit*> vKFInit;

  for(int i=0;i<N;i++){
    KeyFrame* pKF = vScaleGravityKF[i];
    IMUKeyFrameInit* pkfi=new IMUKeyFrameInit(*pKF);
    if(i>0) pkfi->mpPrevKeyFrame=vKFInit[i-1];
    vKFInit.push_back(pkfi);
  }

  SetCopyInitKFs(false);

  // Step 1. / see VIORBSLAM paper IV-A
  // Try to compute initial gyro bias, using optimization with Gauss-Newton
  Vector3d bgest=Optimizer::OptimizeInitialGyroBias<IMUKeyFrameInit>(vKFInit);//nothing changed, just return the optimized result bg*
//   cout<<"bgest: "<<bgest<<endl;

  // Update biasg and pre-integration in LocalWindow(here all KFs).
  for(int i=0;i<N;++i) vKFInit[i]->mbg_=bgest;
  for(int i=1;i<N;++i) vKFInit[i]->ComputePreInt();//so vKFInit[i].mOdomPreIntIMU is based on bg_bar=bgest,ba_bar=0; dbg=0 but dba/ba waits to be optimized

  // Step 2. / See VIORBSLAM paper IV-B
  // Approx Scale and Gravity vector in 'world' frame (first/0th KF's camera frame)
  // Solve A*x=B for x=[s,gw] 4x1 vector, using SVD method
  cv::Mat A=cv::Mat::zeros(3*(N-2),4,CV_32F);//4 unknowns so N must >=4
  cv::Mat B=cv::Mat::zeros(3*(N-2),1,CV_32F);
  cv::Mat I3=cv::Mat::eye(3,3,CV_32F);
  for(int i=0; i<N-2; ++i){
    IMUKeyFrameInit *pKF2=vKFInit[i+1],*pKF3=vKFInit[i+2];
    double dt12 = pKF2->mOdomPreIntIMU.mdeltatij;//deltat12
    double dt23 = pKF3->mOdomPreIntIMU.mdeltatij;
    assert(dt12!=0&&dt23!=0);//now let them not be 0
    assert(dt12>0&&dt23>0);
    // Pre-integrated measurements
    cv::Mat dp12=Converter::toCvMat(pKF2->mOdomPreIntIMU.mpij);//deltap12
    cv::Mat dv12=Converter::toCvMat(pKF2->mOdomPreIntIMU.mvij);
    cv::Mat dp23=Converter::toCvMat(pKF3->mOdomPreIntIMU.mpij);
//     cout<<fixed<<setprecision(6);
//     cout<<"dt12:"<<dt12<<" KF1:"<<vKFInit[i]->mTimeStamp<<" KF2:"<<pKF2->mTimeStamp<<" dt23:"<<dt23<<" KF3:"<<pKF3->mTimeStamp<<endl;
//     cout<<dp12.t()<<" 1id:"<<vScaleGravityKF[i]->mnId<<" 2id:"<<vScaleGravityKF[i+1]->mnId<<" 3id:"<<vScaleGravityKF[i+2]->mnId<<endl;
//     cout<<" Size12="<<pKF2->mOdomPreIntIMU.getlOdom().size()<<" Size23="<<pKF3->mOdomPreIntIMU.getlOdom().size()<<endl;
    // Pose of camera in world frame
    cv::Mat Twc1=vKFInit[i]->mTwc;//Twci for pwci&Rwci, not necessary for clone()
    cv::Mat Twc2=pKF2->mTwc;cv::Mat Twc3=pKF3->mTwc;
    cv::Mat pc1=Twc1.rowRange(0,3).col(3);//pwci
    cv::Mat pc2=Twc2.rowRange(0,3).col(3);cv::Mat pc3=Twc3.rowRange(0,3).col(3);
    cv::Mat Rc1=Twc1.rowRange(0,3).colRange(0,3);//Rwci
    cv::Mat Rc2=Twc2.rowRange(0,3).colRange(0,3);cv::Mat Rc3=Twc3.rowRange(0,3).colRange(0,3);

    // fill A/B matrix: lambda*s + beta*g = gamma(3*1), Ai(3*4)=[lambda beta], (13) in the paper
    cv::Mat lambda=(pc2-pc1)*dt23+(pc2-pc3)*dt12;
    cv::Mat beta=(dt12*dt12*dt23+dt12*dt23*dt23)/2*I3;
    cv::Mat gamma=(Rc1-Rc2)*pcb*dt23+(Rc3-Rc2)*pcb*dt12-Rc2*Rcb*dp23*dt12-Rc1*Rcb*dv12*dt12*dt23+Rc1*Rcb*dp12*dt23;
    lambda.copyTo(A.rowRange(3*i+0,3*i+3).col(0));beta.copyTo(A.rowRange(3*i+0,3*i+3).colRange(1,4));//Ai
    gamma.copyTo(B.rowRange(3*i+0,3*i+3));//gamma/B(i), but called gamma(i) in the paper
    // JingWang tested the formulation in paper, -gamma. Then the scale and gravity vector is -xx, or we can say the papaer missed a minus before γ(i)
  }
  // Use svd to compute A*x=B, x=[s,gw] 4x1 vector
  // A=u*S*vt=u*w*vt, u*w*vt*x=B => x=vt'*winv*u'*B, or we call the pseudo inverse of A/A.inv()=(A.t()*A).inv()*A.t(), in SVD we have A.inv()=v*winv*u.t() where winv is the w with all nonzero term is the reciprocal of the corresponding singular value
  cv::Mat w,u,vt;// Note w is 4x1 vector by SVDecomp()/SVD::compute() not the 4*4(not FULL_UV)/m*n(FULL_UV) singular matrix we stated last line
  cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A);// A is changed in SVDecomp()(just calling the SVD::compute) with cv::SVD::MODIFY_A for speed
  cv::Mat winv=cv::Mat::eye(4,4,CV_32F);
  for(int i=0;i<4;++i){
    if(fabs(w.at<float>(i))<1e-10){//too small in sufficient w meaning the linear dependent equations causing the solution is not unique?
      w.at<float>(i) += 1e-10;
      cerr<<"w(i) < 1e-10, w="<<endl<<w<<endl;
    }
    winv.at<float>(i,i)=1./w.at<float>(i);
  }
  cv::Mat x=vt.t()*winv*u.t()*B;
  double sstar=x.at<float>(0);		// scale should be positive
  cv::Mat gwstar=x.rowRange(1,4);	// gravity should be about ~9.8
  cout<<"gwstar: "<<gwstar.t()<<", |gwstar|="<<cv::norm(gwstar)<<endl;

  // Step 3. / See VIORBSLAM paper IV-C
  // Use gravity magnitude 9.810 as constraint; gIn/^gI=[0;0;1], the normalized gravity vector in an inertial frame
  cv::Mat gIn=cv::Mat::zeros(3,1,CV_32F);gIn.at<float>(2)=1;
  cv::Mat gwn=gwstar/cv::norm(gwstar);//^gw=gw*/||gw*|| / Normalized approx. gravity vecotr in world frame
  cv::Mat gInxgwn=gIn.cross(gwn);double normgInxgwn=cv::norm(gInxgwn);
  cv::Mat vhat=gInxgwn/normgInxgwn;//RwI=Exp(theta*^v), or we can call it vn=(gI x gw)/||gI x gw||
  double theta=std::atan2(normgInxgwn,gIn.dot(gwn));//notice theta*^v belongs to [-Pi,Pi]*|^v| though theta belongs to [0,Pi]
  Matrix3d RWIeig=IMUPreintegrator::Expmap(Converter::toVector3d(vhat)*theta);cv::Mat Rwi=Converter::toCvMat(RWIeig);//RwI
  cv::Mat GI=gIn*IMUData::mdRefG;//gI or GI=^gI*G
  
  // Solve C*x=D for x=[s,dthetaxy,ba] (1+2+3)x1 vector
  cv::Mat C=cv::Mat::zeros(3*(N-2),6,CV_32F);
  cv::Mat D=cv::Mat::zeros(3*(N-2),1,CV_32F);
  for(int i=0; i<N-2; i++){
    IMUKeyFrameInit *pKF2=vKFInit[i+1],*pKF3 = vKFInit[i+2];
    const IMUPreintegrator &imupreint12=pKF2->mOdomPreIntIMU,&imupreint23=pKF3->mOdomPreIntIMU;
    //d means delta
    double dt12=imupreint12.mdeltatij;double dt23=imupreint23.mdeltatij;
    cv::Mat dp12=Converter::toCvMat(imupreint12.mpij);cv::Mat dp23=Converter::toCvMat(imupreint23.mpij);
    cv::Mat dv12=Converter::toCvMat(imupreint12.mvij);cv::Mat Jav12=Converter::toCvMat(imupreint12.mJavij);
    cv::Mat Jap12 = Converter::toCvMat(imupreint12.mJapij);cv::Mat Jap23=Converter::toCvMat(imupreint23.mJapij);
    cv::Mat Twc1=vKFInit[i]->mTwc;//Twci for pwci&Rwci, not necessary for clone()
    cv::Mat Twc2=pKF2->mTwc;cv::Mat Twc3=pKF3->mTwc;
    cv::Mat pc1=Twc1.rowRange(0,3).col(3);//pwci
    cv::Mat pc2=Twc2.rowRange(0,3).col(3);cv::Mat pc3=Twc3.rowRange(0,3).col(3);
    cv::Mat Rc1=Twc1.rowRange(0,3).colRange(0,3);//Rwci
    cv::Mat Rc2=Twc2.rowRange(0,3).colRange(0,3);cv::Mat Rc3=Twc3.rowRange(0,3).colRange(0,3);
    // Stack to C/D matrix; lambda*s + phi(:,0:1)*dthetaxy + zeta*ba = psi, Ci(3*6),Di/psi(3*1)
    cv::Mat lambda=(pc2-pc1)*dt23+(pc2-pc3)*dt12;//3*1
    cv::Mat phi=-(dt12*dt12*dt23+dt12*dt23*dt23)/2*Rwi*SkewSymmetricMatrix(GI);//3*3 note: this has a '-', different to paper
    cv::Mat zeta=Rc2*Rcb*Jap23*dt12+Rc1*Rcb*Jav12*dt12*dt23-Rc1*Rcb*Jap12*dt23;//3*3 notice here is Jav12, paper writes a wrong Jav23
    cv::Mat psi=(Rc1-Rc2)*pcb*dt23+(Rc3-Rc2)*pcb*dt12-Rc2*Rcb*dp23*dt12-Rc1*Rcb*dv12*dt12*dt23//note:  - paper & deltatij^2 in paper means dt12^2*dt23+dt23^2*dt12
    +Rc1*Rcb*dp12*dt23-(dt12*dt12*dt23+dt12*dt23*dt23)/2*(Rwi*GI);//notice here use Rwi*GI instead of gwstar for it's designed for iterative usage
    lambda.copyTo(C.rowRange(3*i+0,3*i+3).col(0));
    phi.colRange(0,2).copyTo(C.rowRange(3*i+0,3*i+3).colRange(1,3));//phi(:,0:1)(3*2) / only the first 2 columns, third term in dtheta is zero, here compute dthetaxy 2x1.
    zeta.copyTo(C.rowRange(3*i+0,3*i+3).colRange(3,6));
    psi.copyTo(D.rowRange(3*i+0,3*i+3));
  }
  // Use svd to compute C*x=D, x=[s,dthetaxy,ba] 6x1 vector
  cv::Mat w2,u2,vt2;// Note w2 is 6x1 vector by SVDecomp()
  cv::SVD::compute(C,w2,u2,vt2,cv::SVD::MODIFY_A);
  cv::Mat w2inv=cv::Mat::eye(6,6,CV_32F);
  for(int i=0;i<6;++i){
    if(fabs(w2.at<float>(i))<1e-10){
      w2.at<float>(i) += 1e-10;
      cerr<<"w2(i) < 1e-10, w="<<endl<<w2<<endl;
    }
    w2inv.at<float>(i,i)=1./w2.at<float>(i);
  }
  cv::Mat y=vt2.t()*w2inv*u2.t()*D;// Then y/x = vt'*winv*u'*D
  double s_=y.at<float>(0);//s*_C, C means IV-C in the paper
  Eigen::Vector3d dthetaeig(y.at<float>(1),y.at<float>(2),0);//small deltatheta/dtheta=[dthetaxy.t() 0].t()
  Eigen::Matrix3d Rwieig_=RWIeig*IMUPreintegrator::Expmap(dthetaeig);//RwI*_C=RwI*_B*Exp(dtheta)
  cv::Mat Rwi_=Converter::toCvMat(Rwieig_);
  Vector3d bastareig=Converter::toVector3d(y.rowRange(3,6));//here bai_bar=0, so dba=ba

  // Record data for analysis
  cv::Mat gwbefore=Rwi*GI,gwafter=Rwi_*GI;//direction of gwbefore is the same as gwstar, but value is different!
  cout<<"Time: "<<pNewestKF->mTimeStamp-mdStartTime<<", sstar: "<<sstar<<", s: "<<s_<<endl;//Debug the frequency & sstar2&sstar
  //<<" bgest: "<<bgest.transpose()<<", gw*(gwafter)="<<gwafter.t()<<", |gw*|="<<cv::norm(gwafter)<<", norm(gwbefore,gwstar)"<<cv::norm(gwbefore.t())<<" "<<cv::norm(gwstar.t())<<endl;
  if (mTmpfilepath.length()>0){//Debug the Rwistar2
    ofstream fRwi(mTmpfilepath+"Rwi.txt");
    fRwi<<Rwieig_(0,0)<<" "<<Rwieig_(0,1)<<" "<<Rwieig_(0,2)<<" "
	<<Rwieig_(1,0)<<" "<<Rwieig_(1,1)<<" "<<Rwieig_(1,2)<<" "
	<<Rwieig_(2,0)<<" "<<Rwieig_(2,1)<<" "<<Rwieig_(2,2)<<endl;
    fRwi.close();
  }
  fbiasg<<pNewestKF->mTimeStamp<<" "<<bgest(0)<<" "<<bgest(1)<<" "<<bgest(2)<<" "<<endl;
  fgw<<pNewestKF->mTimeStamp<<" "<<gwafter.at<float>(0)<<" "<<gwafter.at<float>(1)<<" "<<gwafter.at<float>(2)<<" "
      <<gwbefore.at<float>(0)<<" "<<gwbefore.at<float>(1)<<" "<<gwbefore.at<float>(2)<<" "<<endl;
  fscale<<pNewestKF->mTimeStamp<<" "<<s_<<" "<<sstar<<" "<<endl;//if (mbMonocular) 
  fbiasa<<pNewestKF->mTimeStamp<<" "<<bastareig[0]<<" "<<bastareig[1]<<" "<<bastareig[2]<<" "<<endl;
  fcondnum<<pNewestKF->mTimeStamp<<" "<<w2.at<float>(0)<<" "<<w2.at<float>(1)<<" "<<w2.at<float>(2)<<" "<<w2.at<float>(3)<<" "<<w2.at<float>(4)<<" "<<w2.at<float>(5)<<" "<<endl;

  // ********************************
  // Todo: Add some logic or strategy to confirm init status, VIORBSLAM paper just uses 15 seconds to confirm
  bool bVIOInited = false;
  if(mdStartTime<0) mdStartTime=pNewestKF->mTimeStamp;
  if(pNewestKF->mTimeStamp-mdStartTime>=15){//15s in the paper V-A
    bVIOInited = true;
  }

  //if VIO is initialized with appropriate bg*,s*,gw*,ba*, update the map(MPs' P,KFs' PRVB) like GBA/CorrrectLoop()
  if(bVIOInited){
      // Set NavState , scale and bias for all KeyFrames
      double scale=s_;
      // gravity vector in world frame
      cv::Mat gw;
      {
	unique_lock<mutex> lock(mMutexInitIMU);
	mGravityVec=Rwi_*GI;gw=mGravityVec.clone();
      }
      Vector3d gweig = Converter::toVector3d(gw);

      {// Update the Map needs mutex lock: Stop local mapping, like RunGlobalBundleAdjustment() in LoopClosing.cc
	mpLocalMapper->RequestStop();//same as CorrectLoop(), suspend/stop/freeze LocalMapping thread
	// Wait until Local Mapping has effectively stopped
	while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished()){//if LocalMapping is killed by System::Shutdown(), don't wait any more
	  usleep(1000);
	}

	unique_lock<mutex> lock(mpMap->mMutexMapUpdate);// Get Map Mutex
	//Update KFs' PRVB
	//update the vScaleGravityKF to the current size, and pNewestKF is mpCurrentKeyFrame during the LocalMapping thread is stopped
	vScaleGravityKF=mpMap->GetAllKeyFrames();
	pNewestKF=GetCurrentKeyFrame();
	assert(pNewestKF==vScaleGravityKF.back());//they must be same for we change the set less func. in Map.h
	//recover right scaled Twc&NavState from old unscaled Twc with scale
	for(vector<KeyFrame*>::const_iterator vit=vScaleGravityKF.begin(), vend=vScaleGravityKF.end(); vit!=vend; ++vit){
	  KeyFrame* pKF = *vit;
	  if(pKF->isBad()) continue;
	  //we can SetPose() first even no IMU data
	  cv::Mat Tcw=pKF->GetPose(),Twc=pKF->GetPoseInverse();//we must cache Twc first!
	  cv::Mat tcw=Tcw.rowRange(0,3).col(3)*scale;//right scaled pwc
	  tcw.copyTo(Tcw.rowRange(0,3).col(3));pKF->SetPose(Tcw);//manually SetPose(right scaled Tcw)
	  // Position and rotation of visual SLAM
	  cv::Mat wPc = Twc.rowRange(0,3).col(3);                   // wPc/twc
	  cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);            // Rwc
	  // Set position and rotation of navstate
	  cv::Mat wPb = scale*wPc + Rwc*pcb;//right scaled pwb from right scaled pwc
	  NavState ns;
	  ns.mpwb=Converter::toVector3d(wPb);ns.setRwb(Converter::toMatrix3d(Rwc*Rcb));
	  ns.mbg=bgest;ns.mba=bastareig;//bg* ba*
	  ns.mdbg=ns.mdba=Vector3d::Zero();// Set delta_bias to zero. (only updated during optimization)
	  // Step 4. / See IV-D/(18)/(3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
	  // compute velocity
	  if(pKF!=vScaleGravityKF.back()){
	    KeyFrame* pKFnext=pKF->GetNextKeyFrame();
	    assert(pKFnext&&"pKFnext is NULL");
	    if (pKFnext->GetIMUPreInt().mdeltatij==0){cout<<"time 0"<<endl;continue;}
	    pKF->SetNavStateOnly(ns);//we must update the pKF->mbg&mba before pKFnext->PreIntegration()
	    pKFnext->PreIntegration<IMUData>(pKF);//it's originally based on bi_bar=0, but now it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
	    const IMUPreintegrator imupreint=pKFnext->GetIMUPreInt();//IMU pre-int between pKF ~ pKFnext, though the paper seems to use the vKFInit[k].mOdomPreIntIMU so its dbgi=0 but its dbai=bai, we use more precise bi_bar here
	    double dt=imupreint.mdeltatij;                                		// deltati_i+1
	    cv::Mat dp=Converter::toCvMat(imupreint.mpij);       			// deltapi_i+1
	    //cv::Mat Japij=Converter::toCvMat(imupreint.mJapij);    			// Ja_deltap
	    cv::Mat wPcnext=pKFnext->GetPoseInverse().rowRange(0,3).col(3);		// wPci+1
	    cv::Mat Rwcnext=pKFnext->GetPoseInverse().rowRange(0,3).colRange(0,3);	// Rwci+1
	    cv::Mat vwbi=-1./dt*(scale*(wPc-wPcnext)+(Rwc-Rwcnext)*pcb+dt*dt/2*gw+Rwc*Rcb*(dp));//-1/dt*(pwbi-pwbj+1/2*gw*dt^2+Rwbi*(dp+Japij*dbai)), pwbi=s*pwc+Rwc*pcb, s=sw=swRightScaled_wNow
	    ns.mvwb=Converter::toVector3d(vwbi);
	  }else{
	    // If this is the last KeyFrame, no 'next' KeyFrame exists, use (3) in VOIRBSLAM paper with ba_bar=0,bg_bar=bgest=>dba=ba,dbg=0
	    if (pKF->GetIMUPreInt().mdeltatij==0){cout<<"time 0"<<endl;continue;}
	    KeyFrame* pKFprev=pKF->GetPrevKeyFrame();
	    assert(pKFprev&&"pKFnext is NULL");
	    const IMUPreintegrator imupreint=pKF->GetIMUPreInt();//notice it's based on bi_bar=[bg* ba*], so dbgi=dbai=0
	    double dt=imupreint.mdeltatij;
	    NavState nsprev=pKFprev->GetNavState();
	    ns.mvwb=nsprev.mvwb+gweig*dt+nsprev.mRwb*(imupreint.mvij);//vwbj=vwbi+gw*dt+Rwbi*(dvij+Javij*dbai)
	  }
	  pKF->SetNavStateOnly(ns);//now ns also has the right mvwb
	}
	//Update MPs' Position
	vector<MapPoint*> vpMPs=mpMap->GetAllMapPoints();//we don't change the vpMPs[i] but change the *vpMPs[i]
	for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; ++vit) (*vit)->UpdateScale(scale);
	//Now every thing in Map is right scaled & mGravityVec is got
	SetVINSInited(true);
	SetFirstVINSInited(true);
	
	mpMap->InformNewBigChange();//used to check the SLAM's state
	mpLocalMapper->Release();//recover LocalMapping thread, same as CorrectLoop()
	std::cout<<std::endl<<"... Map scale & NavState updated ..."<<std::endl<<std::endl;
      }

      // Run global BA after inited
      unsigned long nLoopKF=pNewestKF->mnId;//does it need to be set noterase?
      Optimizer::GlobalBundleAdjustmentNavStatePRV(mpMap,mGravityVec,10,NULL,nLoopKF,false);
      cerr<<red<<"finish global BA after vins init"<<white<<endl;
      
      {//the same as LoopClosing::RunGlobalBundleAdjustment()
	mpLocalMapper->RequestStop();//same as CorrectLoop(), suspend/stop/freeze LocalMapping thread
	// Wait until Local Mapping has effectively stopped
	while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())//if LocalMapping is killed by System::Shutdown(), don't wait any more
	{
	    usleep(1000);
	}
	
	// Get Map Mutex
	unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
	// Correct keyframes starting at map first keyframe(id 0)
	list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());
	//propagate the correction through the spanning tree(root is always id0 KF)
	while(!lpKFtoCheck.empty())//if the propagation is not over (notice mpMap cannot be reset for LocalMapping is stopped)
	{
	    KeyFrame* pKF = lpKFtoCheck.front();//for RGBD, lpKFtoCheck should only have one KF initially
	    const set<KeyFrame*> sChilds = pKF->GetChilds();
	    cv::Mat Twc = pKF->GetPoseInverse();
	    for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
	    {
		KeyFrame* pChild = *sit;
		if(pChild->mnBAGlobalForKF!=nLoopKF)//if child is not GBA optimized by mpCurrentKF/it must be the new KFs created by LocalMapping thread during GBA
		{
		    cv::Mat Tchildc = pChild->GetPose()*Twc;//Tchildw*Tw0=Tchild0
		    pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA; Tchild0*T0w(corrected)=Tchildw(corrected)
		    pChild->mnBAGlobalForKF=nLoopKF;//so now its child KF' Pose can seem to be corrected by GBA

		    // Set NavStateGBA and correct the PR&V
		    pChild->mNavStateGBA = pChild->GetNavState();//Tb_old_w
		    Matrix3d Rw1 = pChild->mNavStateGBA.getRwb();Vector3d Vw1 = pChild->mNavStateGBA.mvwb;//Rwb_old&wVwb_old
		    cv::Mat TwbGBA = Converter::toCvMatInverse(Frame::mTbc*pChild->mTcwGBA);//TbwGBA.t()
		    Matrix3d RwbGBA=Converter::toMatrix3d(TwbGBA.rowRange(0,3).colRange(0,3));
		    pChild->mNavStateGBA.setRwb(RwbGBA);
		    pChild->mNavStateGBA.mpwb=Converter::toVector3d(TwbGBA.rowRange(0,3).col(3));
		    pChild->mNavStateGBA.mvwb=RwbGBA*Rw1.transpose()*Vw1;//Vwb_new=wVwb_new=Rwb_new*bVwb=Rwb_new*Rb_old_w*wVwb_old=Rwb2*Rwb1.t()*wV1
		}//now the child is optimized by GBA
		lpKFtoCheck.push_back(pChild);
	    }

	    pKF->mTcwBefGBA = pKF->GetPose();//record the old Tcw
	    pKF->SetNavState(pKF->mNavStateGBA);//update all KFs' Pose to GBA optimized Tbw&Tcw, including UpdatePoseFromNS()&&SetPose(pKF->mTcwGBA), not necessary to update mNavStateBefGBA for unused in MapPoints' correction
	    lpKFtoCheck.pop_front();
	}

	// Correct MapPoints
	const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
	for(size_t i=0; i<vpMPs.size(); i++)//all MPs in mpMap
	{
	    MapPoint* pMP = vpMPs[i];

	    if(pMP->isBad())
		continue;

	    if(pMP->mnBAGlobalForKF==nLoopKF)//if this MP is GBA optimized
	    {
		// If optimized by Global BA, just update
		pMP->SetWorldPos(pMP->mPosGBA);//update all (old)MPs' Pos to GBA optimized Pos
	    }
	    else//new MPs created by Tracking/LocalMapping thread during GBA
	    {
		// Update according to the correction of its reference keyframe
		KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

		if(pRefKF->mnBAGlobalForKF!=nLoopKF)//I think it should be false for it's propagated through spanning tree,need test
		    continue;

		// Map to non-corrected camera
		cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);//old Rcw
		cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);//old tcw
		cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;//Xc=(Tcw(old)*Pw(old))(0:2)

		// Backproject using corrected camera
		cv::Mat Twc = pRefKF->GetPoseInverse();//new Twc
		cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);//new Rwc
		cv::Mat twc = Twc.rowRange(0,3).col(3);//new twc

		pMP->SetWorldPos(Rwc*Xc+twc);//update all (new)MPs' Pos to GBA optimized Pos/Pw(new)=Twc(new)*Pc
	    }
	}       

	mpMap->InformNewBigChange();//used to check the SLAM's state
	mpLocalMapper->Release();//recover LocalMapping thread, same as CorrectLoop()
	
	cout <<red<< "Map updated in IMU Initialization!" << white<<endl;//if Initial GBA/loop correction successed, this word should appear!
	SetInitGBAFinish(true);//allow LoopClosing to start
      }
  }

  for(int i=0;i<N;i++){//delete the newed pointer
    if(vKFInit[i]) delete vKFInit[i];
  }
  return bVIOInited;
}
  
}