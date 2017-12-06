//created by zzh
#ifndef ODOMDATA_H
#define ODOMDATA_H

#define TRACK_WITH_IMU

//for Jacobi calculation & member data
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ORB_SLAM2{

using namespace Eigen;

class IMUDataBase
{ 
  static double mdMultiplyG;//IMU.dMultiplyG for getting right scaled accelerate
public:
  static double mdRefG;//referenced G for IV-C in VIORBSLAM paper
  static Matrix3d mSigmagd,mSigmaad,mSigmabg,mSigmaba;//b means bias/Brownian motion(Random walk), g means gyroscope, a means accelerator, d means discrete, Sigma means Covariance Matrix
  static double mInvSigmabg2,mInvSigmaba2;//when mSigmabi is always diagonal matrix, use this to speed up infomation matrix calculation
  double mtm;//timestamp of IMU data
  Vector3d ma,mw;//accelerate ba~b(t)(m/s^2) & rotation velocity bw~b(t)(rad/s)
  
  IMUDataBase():mtm(-1){}//do nothing, tm==-1 means it's an invalid IMUData
  IMUDataBase(const double pdata[6],const double &tm):mtm(tm),ma(pdata),mw(pdata+3){
    ma*=mdMultiplyG;//for my IMU dataset uses 1 meaning standard G / 9.80665
  }//pdata 6*1: axyz then wxyz
  static void SetParam(const double sigma2[4],double dmultiplyG=1.0){//gd,ad,bgd,bad
    mSigmagd=Matrix3d::Identity()*sigma2[0],mSigmaad=Matrix3d::Identity()*sigma2[1];
    mSigmabg=Matrix3d::Identity()*sigma2[2],mSigmaba=Matrix3d::Identity()*sigma2[3];mInvSigmabg2=1./sigma2[2];mInvSigmaba2=1./sigma2[3];
    mdMultiplyG=dmultiplyG;
  }//for dynamic binding
  //= will use Vector3d's deep =, copy constructor is also deep
  virtual ~IMUDataBase(){}//for Derived class
  
};
class IMUDataDerived:public IMUDataBase
{ 
  Matrix3d skew(const Vector3d&v);
  
public:
  static Matrix3d mSigmaI;// Sigma etawi of quaternionIMU(qIMU), I/i means Inertial here
  Quaterniond quat;//quaternion of IMUData added

  IMUDataDerived(const double* pdata,const double &tm):quat(pdata){mtm=tm;quat.normalize();}//pdata 4*1: qxyzw
  static void SetParam(const Matrix3d &sigmai,const double sigma2[4],double dmultiplyG=1.0){mSigmaI=sigmai;IMUDataBase::SetParam(sigma2,dmultiplyG);}//gd,ad,bgd,bad, rewrite for virtual so it's not reload
  Matrix3d getJacoright();
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#ifndef TRACK_WITH_IMU
typedef IMUDataDerived IMUData;
#else
typedef IMUDataBase IMUData;
#endif

class EncData{
public:
  static double mvscale;//encoder coefficient to m/s
  static double mrc;//rc: 2 differential driving wheels' distance
  static Matrix2d mSigmad;// mSigmad Sigma etad of Enc
  double mv[2],mtm;//v[0]=vl,v[1]=vr(m/s),timestamp of EncoderData
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EncData(const double v[2],const double &tm);
  static void SetParam(const double &vscale,const double &rc,const Matrix2d &sigmad){mvscale=vscale;mrc=rc;mSigmad=sigmad;}
  //need to overlap the default version to realize deep copy
  EncData(const EncData& encdata):mv{encdata.mv[0],encdata.mv[1]},mtm(encdata.mtm){}
  EncData& operator=(const EncData& encdata){mv[0]=encdata.mv[0];mv[1]=encdata.mv[1];mtm=encdata.mtm;return *this;}
  
};

}
    
#endif