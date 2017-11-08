//created by zzh
#include "OdomData.h"

namespace ORB_SLAM2{

using namespace Eigen;
Matrix3d IMUData::skew(const Vector3d&v)
{
  Matrix3d m;
  m.fill(0.);
  m(0,1)  = -v(2);
  m(0,2)  =  v(1);
  m(1,2)  = -v(0);
  m(1,0)  =  v(2);
  m(2,0) = -v(1);
  m(2,1) = v(0);
  return m;
}
IMUData::IMUData(const double* data_,const double tm_,const char type_):tm(tm_),type(type_){
  switch(type_){//type_==-1 do nothing, it's a invalid IMUData
    case 0:
      for (int i=0;i<4;++i) quat[i]=data_[i];
      break;
    case 1:
      for (int i=0;i<3;++i){ a[i]=data_[i];w[i]=data_[i+3];}
      break;
  }
}
IMUData& IMUData::operator=(const IMUData& data){
  for (int i=0;i<4;++i) quat[i]=data.quat[i];
  tm=data.tm;type=data.type;
  for (int i=0;i<3;++i){ a[i]=data.a[i];w[i]=data.w[i];}
  return *this;
}
Eigen::Matrix3d IMUData::getJacoright(){
  Eigen::AngleAxisd angaxi=Eigen::AngleAxisd(Eigen::Quaterniond(quat));//angleaxisi need explicit conversion!
  double th=angaxi.angle();
  Eigen::Matrix3d skewa=skew(angaxi.axis());
  if (th<1E-5){
    return (Eigen::Matrix3d::Identity()-th*skewa/2);
  }
  return (Eigen::Matrix3d::Identity()-(1-cos(th))/th*skewa+(1-sin(th)/th)*skewa*skewa);
}

double EncData::mstvscale=1;
EncData::EncData(const double* v_,const double tm_):tm(tm_){
  v[0]=v_[0]*mstvscale;v[1]=v_[1]*mstvscale;
}

}
