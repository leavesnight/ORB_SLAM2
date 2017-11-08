//created by zzh
#ifndef ODOMDATA_H
#define ODOMDATA_H

//for Jacobi calculation
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ORB_SLAM2{

class IMUData
{
  Eigen::Matrix3d skew(const Eigen::Vector3d&v);
public:
  double quat[4],tm;//quaternion,timestamp of IMUData
  double a[3],w[3];//accelerate bab(t)(m/s^2) & rotation velocity bwb(t)(rad/s)
  char type;//0 for quat,1 for a&w
  
public:
  IMUData():tm(-1),type(-1){}
  IMUData(const double* data_,const double tm_,const char type_);
  IMUData& operator=(const IMUData& data);
  Eigen::Matrix3d getJacoright();
};

class EncData{
  static double mstvscale;//encoder coefficient to m/s
  
public:
  double v[2],tm;//v[0]=vl,v[1]=vr(m/s),timestamp of EncoderData
  
public:
  EncData(const double* v_,const double tm_);
  static void Setvscale(double vscale){mstvscale=vscale;}
};

}
    
#endif