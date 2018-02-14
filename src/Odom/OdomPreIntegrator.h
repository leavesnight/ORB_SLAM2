//created by zzh, inspired by Jing Wang
#ifndef ODOMPREINTEGRATOR_H
#define ODOMPREINTEGRATOR_H

#include <list>
#include "OdomData.h"
#include "so3.h"//for IMUPreIntegratorBase::PreIntegration

#include <iostream>

namespace ORB_SLAM2{

using namespace Eigen;
  
template<class _OdomData>
class OdomPreIntegratorBase{//base class
  OdomPreIntegratorBase(const OdomPreIntegratorBase &pre){}//don't want the list to be copied (e.g. by derived class)
  OdomPreIntegratorBase& operator=(const OdomPreIntegratorBase &other){;return *this;}//do nothing, don't want the list to be assigned in any situation, this makes the derived class unable to use default =!
  
protected:
  listeig(_OdomData) mlOdom;//for IMUPreIntegrator: IMU list
  
public:
  double mdeltatij;//0 means not preintegrated
  
  OdomPreIntegratorBase():mdeltatij(0){}
  //though copy constructor/operator = already deep, please don't copy the list when preintegration is not related to the statei...j
  virtual ~OdomPreIntegratorBase(){}
  // Odom PreIntegration List Setting
  virtual void SetPreIntegrationList(const typename listeig(_OdomData)::const_iterator &begin,typename listeig(_OdomData)::const_iterator pback){
    mlOdom.clear();
    mlOdom.insert(mlOdom.begin(),begin,++pback);
  }
  const listeig(_OdomData)& getlOdom(){return mlOdom;}//the list of Odom, for KFCulling()
  // Odom PreIntegration
  virtual void PreIntegration(const double timeStampi,const double timeStampj){assert(0&&"You called an empty virtual function!!!");}//cannot use =0 for we allow transformed in derived class
  
  // normalize to avoid numerical error accumulation
  inline Quaterniond normalizeRotationQ(const Quaterniond& r) const
  {
    Quaterniond _r(r);
    if (_r.w()<0)//is this necessary?
    {
	_r.coeffs() *= -1;
    }
    return _r.normalized();
  }
  inline Matrix3d normalizeRotationM(const Matrix3d& R) const
  {
    Quaterniond qr(R);
    return normalizeRotationQ(qr).toRotationMatrix();
  }
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Eigen::Matrix<double, 6, 1> Vector6d;

//next derived classes don't use operator=!
class EncPreIntegrator:public OdomPreIntegratorBase<EncData>{
  //mlOdom: mlOdomEnc list for vl,vr& its own timestamp
public:
  Vector6d mdelxEij;// delta~Phiij(3*1),delta~pij(3*1) from Encoder PreIntegration, 6*1*float
  Matrix6d mSigmaEij;// by Enc, 6*6*float
  
  EncPreIntegrator():mdelxEij(Vector6d::Zero()),mSigmaEij(Matrix6d::Zero()){}
  EncPreIntegrator(const EncPreIntegrator &pre):mdelxEij(pre.mdelxEij),mSigmaEij(pre.mSigmaEij){mdeltatij=pre.mdeltatij;}//don't copy list!
  EncPreIntegrator& operator=(const EncPreIntegrator &pre){
    mdeltatij=pre.mdeltatij;//don't copy list!
    mdelxEij=pre.mdelxEij;mSigmaEij=pre.mSigmaEij;
    return *this;
  }
  void PreIntegration(const double &timeStampi,const double &timeStampj,
		      const listeig(EncData)::const_iterator &iterBegin,const listeig(EncData)::const_iterator &iterEnd);//rewrite
  void PreIntegration(const double &timeStampi,const double &timeStampj){PreIntegration(timeStampi,timeStampj,mlOdom.begin(),mlOdom.end());}//rewrite, inline
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Eigen::Matrix<double, 9, 9> Matrix9d;

template<class IMUDataBase>
class IMUPreIntegratorBase:public OdomPreIntegratorBase<IMUDataBase>{//refer the IMUPreintergrator.cpp by JingWang, so use PVR/PRV Cov.
public:
  Matrix3d mRij;//deltaR~ij(bgi_bar) by awIMU, 3*3*float/delta~Rbibj
  Vector3d mvij,mpij;//deltav~ij,deltap~ij(bi_bar)
  Matrix9d mSigmaijPRV;//Cov_p_Phi_v_ij, a bit different with paper for convenience
  Matrix9d mSigmaij;//Cov_p_v_Phi_ij, opposite order with the paper
  // jacobian of delta measurements w.r.t bias of gyro/acc
  Matrix3d mJgpij;	// position / gyro, Jgdeltapij(bi_bar) in VIORBSLAM paper, par(deltapij)/par(bgi).t()(bi_bar) in Preintegration Paper, we may drop .t() later
  Matrix3d mJapij;	// position / acc
  Matrix3d mJgvij;	// velocity / gyro
  Matrix3d mJavij;	// velocity / acc
  Matrix3d mJgRij;	// rotation / gyro
  //Vector3d mbgij,mbaij;//bg(ti)=b_bar_gi+db_gi, bg~ij not exists, neither do ba~ij
  //Matrix3d mSigmabgd,mSigmabad;//should be deltatij*Cov(eta_bg),deltatij*Cov(eta_ba)
  
  IMUPreIntegratorBase():mRij(Matrix3d::Identity()),mvij(0,0,0),mpij(0,0,0),mSigmaijPRV(Matrix9d::Zero()),mSigmaij(Matrix9d::Zero()){
    mJgpij.setZero();mJapij.setZero();mJgvij.setZero();mJavij.setZero();mJgRij.setZero();
  }
  IMUPreIntegratorBase(const IMUPreIntegratorBase &pre):mRij(pre.mRij),mvij(pre.mvij),mpij(pre.mpij),mSigmaijPRV(pre.mSigmaijPRV),mSigmaij(pre.mSigmaij),
  mJgpij(pre.mJgpij),mJapij(pre.mJapij),mJgvij(pre.mJgvij),mJavij(pre.mJavij),mJgRij(pre.mJgRij){
    this->mdeltatij=pre.mdeltatij;//2-phase name lookup used in Derived template class
  }//don't copy list!
  IMUPreIntegratorBase& operator=(const IMUPreIntegratorBase &pre){
    this->mdeltatij=pre.mdeltatij;//don't copy list!
    this->mRij=pre.mRij,this->mvij=pre.mvij,this->mpij=pre.mpij,this->mSigmaijPRV=pre.mSigmaijPRV,this->mSigmaij=pre.mSigmaij,
    this->mJgpij=pre.mJgpij,this->mJapij=pre.mJapij,this->mJgvij=pre.mJgvij,this->mJavij=pre.mJavij,this->mJgRij=pre.mJgRij;
    return *this;
  }
  virtual ~IMUPreIntegratorBase(){}
  
  void PreIntegration(const double &timeStampi,const double &timeStampj,const Vector3d &bgi_bar,const Vector3d &bai_bar,
		      const typename listeig(IMUDataBase)::const_iterator &iterBegin,const typename listeig(IMUDataBase)::const_iterator &iterEnd);//rewrite, like override but different
  void PreIntegration(const double &timeStampi,const double &timeStampj,const Vector3d &bgi_bar,const Vector3d &bai_bar){//inline
    PreIntegration(timeStampi,timeStampj,bgi_bar,bai_bar,this->mlOdom.begin(),this->mlOdom.end());
  }//rewrite
  // incrementally update 1)delta measurements, 2)jacobians, 3)covariance matrix
  void update(const Vector3d& omega, const Vector3d& acc, const double& dt);//don't allow dt<0!
  
  // reset to initial state
  void reset(){
    mRij.setIdentity();mvij.setZero();mpij.setZero();mSigmaijPRV.setZero();mSigmaij.setZero();
    mJgpij.setZero();mJapij.setZero();mJgvij.setZero();mJavij.setZero();mJgRij.setZero();
    this->mdeltatij=0;//very important!
  }
  
  // exponential map from vec3 to mat3x3 (Rodrigues formula)
  static Matrix3d Expmap(const Vector3d& v){//here is inline, but defined in .cpp is ok for efficiency due to copy elision(default gcc -O2 uses it) when return a temporary variable(NRVO/URVO)
    return Sophus::SO3::exp(v).matrix();//here is URVO
  }
};
//when template<>: specialized definition should be defined in .cpp(avoid redefinition) or use inline/static(not good) in .h and template func. in template class can't be specialized(only fully) when its class is not fully specialized
template<class IMUDataBase>
void IMUPreIntegratorBase<IMUDataBase>::PreIntegration(const double &timeStampi,const double &timeStampj,const Vector3d &bgi_bar,const Vector3d &bai_bar,
						       const typename listeig(IMUDataBase)::const_iterator &iterBegin,const typename listeig(IMUDataBase)::const_iterator &iterEnd){
  //TODO: refer to the code by JingWang
  if (iterBegin!=iterEnd&&timeStampi<timeStampj){//default parameter = !mlOdom.empty(); timeStampi may >=timeStampj for Map Reuse
    // Reset pre-integrator first
    reset();
    // remember to consider the gap between the last KF and the first IMU
    // integrate each imu
    for (typename listeig(IMUDataBase)::const_iterator iterj=iterBegin;iterj!=iterEnd;){
      typename listeig(IMUDataBase)::const_iterator iterjm1=iterj++;//iterj-1
      
      // delta time
      double dt,tj,tj_1;
      if (iterjm1==iterBegin) tj_1=timeStampi; else tj_1=iterjm1->mtm;
      if (iterj==iterEnd) tj=timeStampj; else{ tj=iterj->mtm;assert(tj-tj_1>=0);}
      dt=tj-tj_1;
      if (dt==0) continue;//for we use [nearest imu data at timeStampi, nearest but <=timeStampj] or [/(timeStampi,timeStampj], when we concate them in KeyFrameCulling(), dt may be 0
      if (dt>1.5){ this->mdeltatij=0;std::cout<<"CheckIMU!!!"<<std::endl;return;}//for Map Reuse, the edge between last KF of the map and 0th KF of 2nd SLAM should have no odom info (20frames,>=10Hz, 1.5s<=2s is enough for not using MAP_REUSE_RELOC)
      
      //selete/design measurement_j-1
      const IMUDataBase& imu=*iterjm1;//imuj-1 for w~j-1 & a~j-1 chooses imu(tj-1), maybe u can try (imu(tj-1)+imu(tj))/2 or other filter here
      
      // update pre-integrator
      update(imu.mw-bgi_bar,imu.ma-bai_bar,dt);
    }
  }
}
template<class IMUDataBase>
void IMUPreIntegratorBase<IMUDataBase>::update(const Vector3d& omega, const Vector3d& acc, const double& dt){
  using namespace Sophus;
  using namespace Eigen;
  double dt2div2=dt*dt/2;
  Matrix3d dR=Expmap(omega*dt);//Exp((w~j-1 - bgi_bar)*dtj-1j)=delta~Rj-1j
  Matrix3d Jr=SO3::JacobianR(omega*dt);//Jrj-1=Jr(dtj-1j*(w~j-1 - bgi_bar))
  Matrix3d skewa=SO3::hat(acc);//(~aj-1 - bai_bar)^

  //see paper On-Manifold Preintegration (63), notice PRV is different from paper RVP, but the BgBa is the same(A change row&col, B just change row)
  // err_k+1 = A*err_k + Bg*err_gyro + Ba*err_acc; or Bj-1=[Bg Ba],Aj-1=A 
  Matrix3d I3x3 = Matrix3d::Identity();
  Matrix<double,9,9> A = Matrix9d::Identity();
  A.block<3,3>(3,3) = dR.transpose();
  A.block<3,3>(6,3) = -mRij*skewa*dt;
  A.block<3,3>(0,3) = -mRij*skewa*dt2div2;
  A.block<3,3>(0,6) = I3x3*dt;
  Matrix<double,9,3> Bg = Matrix<double,9,3>::Zero();
  Bg.block<3,3>(3,0) = Jr*dt;
  Matrix<double,9,3> Ba = Matrix<double,9,3>::Zero();
  Ba.block<3,3>(6,0) = mRij*dt;
  Ba.block<3,3>(0,0) = mRij*dt2div2;
  mSigmaijPRV=A*mSigmaijPRV*A.transpose()+Bg*IMUDataBase::mSigmagd*Bg.transpose()+Ba*IMUDataBase::mSigmaad*Ba.transpose();//notice using Sigma_eta_g/a_d here
  //the order is opposite(row&col order) for A, opposite row for B
  A = Matrix9d::Identity();
  A.block<3,3>(6,6) = dR.transpose();
  A.block<3,3>(3,6) = -mRij*skewa*dt;
  A.block<3,3>(0,6) = -mRij*skewa*dt2div2;
  A.block<3,3>(0,3) = I3x3*dt;
  Bg = Matrix<double,9,3>::Zero();
  Bg.block<3,3>(6,0) = Jr*dt;
  Ba = Matrix<double,9,3>::Zero();
  Ba.block<3,3>(3,0) = mRij*dt;
  Ba.block<3,3>(0,0) = mRij*dt2div2;
  mSigmaij=A*mSigmaij*A.transpose()+Bg*IMUDataBase::mSigmagd*Bg.transpose()+Ba*IMUDataBase::mSigmaad*Ba.transpose();
  
  //see the same paper (69) & use similar iterative rearrange method (59)
  // jacobian of delta measurements w.r.t bias of gyro/acc, for motion_update_with_dbi & residual error & J_error_dxi,xj calculation
  // update P first, then V, then R for using ij as ij-1 term
  mJapij += mJavij*dt - mRij*dt2div2;//mRij here is delta~Rij-1 before its updation
  mJgpij += mJgvij*dt - mRij*skewa*mJgRij*dt2div2;
  mJavij += -mRij*dt;
  mJgvij += -mRij*skewa*mJgRij*dt;//notice except mJgRij use dR, the other Jxxij use mRij!
  mJgRij = dR.transpose()*mJgRij - Jr*dt;//like (59): JgRij=delta~Rj-1j.t()*JgRij-1 - Jrj-1*dtj-1j, the left incremental formula is easy to get for there's no j label
  
  //see paper On-Manifold Preintegration (35~37)
  mpij+=mvij*dt+mRij*(acc*dt2div2);//delta~pij=delta~pij-1 + delta~vij-1*dtj-1j + 1/2*delta~Rij-1*(~aj-1 - bai_bar)*dtj-1j^2
  mvij+=mRij*(acc*dt);//here mRij=mRij-1, delta~vij=delta~vij-1 + delta~Rij-1 * (~aj-1 - bai_bar)*dtj-1j
  // normalize rotation, in case of numerical error accumulation
  mRij=this->normalizeRotationM(mRij*dR);//here omega=(w~k-bgi_bar)(k=j-1), deltaR~ij(bgi_bar)=deltaRij-1(bgi_bar) * Exp((w~j-1 - bgi_bar)*dtj-1j)
  
  this->mdeltatij+=dt;
}

class IMUPreIntegratorDerived:public IMUPreIntegratorBase<IMUDataDerived>{
public:
  Matrix3d mdelxRji;// delta~Rij.t() from qIMU PreIntegration, 3*3*float
  Matrix3d mSigmaPhiij;// SigmaPhiij by qIMU, 3*3*float

  IMUPreIntegratorDerived():mdelxRji(Matrix3d::Identity()),mSigmaPhiij(Matrix3d::Zero()){}
  void SetPreIntegrationList(const listeig(IMUDataDerived)::const_iterator &begin,const listeig(IMUDataDerived)::const_iterator &pback){//rewrite, will override the base class one
    this->mlOdom.clear();
    this->mlOdom.push_front(*begin);this->mlOdom.push_back(*pback);
  }
  void PreIntegration(const double &timeStampi,const double &timeStampj);//rewrite
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#ifndef TRACK_WITH_IMU
typedef IMUPreIntegratorDerived IMUPreintegrator;
#else
typedef IMUPreIntegratorBase<IMUDataBase> IMUPreintegrator;
#endif

}

#endif