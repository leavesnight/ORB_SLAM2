//created by zzh
#ifndef ODOMDATA_H
#define ODOMDATA_H

#ifdef NDEBUG//assert in Release, should be put at the last include in .cpp
  #undef NDEBUG
  #include <assert.h>
  //#define NDEBUG//else you have to annote this line
#else
  #include <assert.h>
#endif

#define TRACK_WITH_IMU

//for Jacobi calculation & member data
#include <Eigen/Core>
#include <Eigen/Geometry>

//for typedef listEncData
#include <list>

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
//   static void SetParam(const Matrix3d sigma[4],double dmultiplyG=1.0){//gd,ad,bg,ba; for LoadMap()
//     mSigmagd=sigma[0],mSigmaad=sigma[1];mSigmabg=sigma[2],mSigmaba=sigma[3];
//     mInvSigmabg2=1./sigma[2](0,0);mInvSigmaba2=1./sigma[3](0,0);
//     mdMultiplyG=dmultiplyG;
//   }
  //= will use Vector3d's deep =, copy constructor is also deep
  virtual ~IMUDataBase(){}//for Derived class
  
  //for Load/SaveMap()
  virtual bool read(std::istream &is){
    is.read((char*)&mtm,sizeof(mtm));
    is.read((char*)ma.data(),ma.size()*sizeof(Vector3d::Scalar));
    is.read((char*)mw.data(),mw.size()*sizeof(Vector3d::Scalar));
    return is.good();
  }
  virtual bool write(std::ostream &os) const{
    os.write((char*)&mtm,sizeof(mtm));
    os.write((char*)ma.data(),ma.size()*sizeof(Vector3d::Scalar));
    os.write((char*)mw.data(),mw.size()*sizeof(Vector3d::Scalar));
    return os.good();
  }
  static bool readParam(std::istream &is){
    is.read((char*)&mdMultiplyG,sizeof(mdMultiplyG));is.read((char*)&mdRefG,sizeof(mdRefG));
    is.read((char*)mSigmagd.data(),mSigmagd.size()*sizeof(Vector3d::Scalar));
    is.read((char*)mSigmaad.data(),mSigmaad.size()*sizeof(Vector3d::Scalar));
    is.read((char*)mSigmabg.data(),mSigmabg.size()*sizeof(Vector3d::Scalar));
    is.read((char*)mSigmaba.data(),mSigmaba.size()*sizeof(Vector3d::Scalar));
    mInvSigmabg2=1./mSigmabg(0,0);mInvSigmaba2=1./mSigmaba(0,0);
    return is.good();
  }
  static bool writeParam(std::ostream &os){
    os.write((char*)&mdMultiplyG,sizeof(mdMultiplyG));os.write((char*)&mdRefG,sizeof(mdRefG));
    os.write((char*)mSigmagd.data(),mSigmagd.size()*sizeof(Vector3d::Scalar));
    os.write((char*)mSigmaad.data(),mSigmaad.size()*sizeof(Vector3d::Scalar));
    os.write((char*)mSigmabg.data(),mSigmabg.size()*sizeof(Vector3d::Scalar));
    os.write((char*)mSigmaba.data(),mSigmaba.size()*sizeof(Vector3d::Scalar));
    return os.good();
  }
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

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

class EncData{
public:
  static double mvscale;//encoder coefficient to m/s
  static double mrc;//rc: 2 differential driving wheels' distance
  static Matrix2d mSigmad;// mSigmad Sigma etad of Enc
  static Matrix6d mSigmamd;// mmSigmamd Sigma etamd of Enc
  double mv[2],mtm;//v[0]=vl,v[1]=vr(m/s),timestamp of EncoderData
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EncData():mtm(-1){}//do nothing, tm==-1 means it's an invalid EncData
  EncData(const double v[2],const double &tm);
  static void SetParam(const double &vscale,const double &rc,const Matrix2d &sigmad,const Matrix6d &sigmamd){
    mvscale=vscale;mrc=rc;mSigmad=sigmad;mSigmamd=sigmamd;
  }
  //need to overlap the default version to realize deep copy
  EncData(const EncData& encdata):mv{encdata.mv[0],encdata.mv[1]},mtm(encdata.mtm){}
  EncData& operator=(const EncData& encdata){mv[0]=encdata.mv[0];mv[1]=encdata.mv[1];mtm=encdata.mtm;return *this;}//useless for array name is rvalue, always be deep copied!
  
  //for Load/SaveMap()
  bool read(std::istream &is){
    is.read((char*)mv,sizeof(mv));//for mv[N] not double*!
    is.read((char*)&mtm,sizeof(mtm));
    return is.good();
  }
  bool write(std::ostream &os) const{
    os.write((char*)mv,sizeof(mv));//for mv[N] not double*!
    os.write((char*)&mtm,sizeof(mtm));
    return os.good();
  }
  static bool readParam(std::istream &is){
    is.read((char*)&mvscale,sizeof(mvscale));
    is.read((char*)&mrc,sizeof(mrc));
    is.read((char*)mSigmad.data(),mSigmad.size()*sizeof(Vector3d::Scalar));
    is.read((char*)mSigmamd.data(),mSigmamd.size()*sizeof(Vector3d::Scalar));
    return is.good();
  }
  static bool writeParam(std::ostream &os){
    os.write((char*)&mvscale,sizeof(mvscale));
    os.write((char*)&mrc,sizeof(mrc));
    os.write((char*)mSigmad.data(),mSigmad.size()*sizeof(Vector3d::Scalar));
    os.write((char*)mSigmamd.data(),mSigmamd.size()*sizeof(Vector3d::Scalar));
    return os.good();
  }
};

#define listeig(EncData) std::list<EncData,Eigen::aligned_allocator<EncData> >

}
    
#endif
