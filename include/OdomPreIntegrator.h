//created by zzh, inspired by Jing Wang
#ifndef ODOMPREINTEGRATOR_H
#define ODOMPREINTEGRATOR_H

#include <list>
#include <opencv2/core/core.hpp>
#include "OdomData.h"

namespace ORB_SLAM2{
  
class OdomPreIntegrator{
  //rc
  static double mrc;//2 differential driving wheels' distance
  //Sigma etad of Enc & etawi of quaternionIMU(qIMU)
  static cv::Mat mSigmad,mSigmaI;
  std::list<EncData> mlOdomEnc;//list for vl,vr & its own timestamp
  std::list<IMUData> mlOdomIMU;//IMU list
public:
  std::vector<float> mdelxqIijT;//delta~Rij.t()/q~ij* from qIMU PreIntegration, 4*1*float
  cv::Mat mdelxEij;//delta~Phiijz,delta~pij(2*1) from Encoder PreIntegration, 3*1*float
  cv::Mat mSigmaqIij;//SigmaqIij by qIMU, 3*3*float
  cv::Mat mSigmaEij;//SigmaEij by Enc, 3*3*float
public:
  //initialize the static member from the setting file
  static void SetParam(const double &rc,cv::Mat Sigmad,cv::Mat SigmaI){mrc=rc;mSigmad=Sigmad.clone();mSigmaI=SigmaI.clone();}
  // Odom PreIntegration
  template <class _OdomData>
  void SetPreIntegrationList(typename std::list<_OdomData>::iterator &begin,typename std::list<_OdomData>::iterator pback){
    mlOdomEnc.clear();
    mlOdomEnc.insert(mlOdomEnc.begin(),begin,++pback);
  }
  void PreIntegration(const double timeStampi,const double timeStamp);
};
template <>//specialized template should be declared in the same file
void OdomPreIntegrator::SetPreIntegrationList<IMUData>(std::list<IMUData>::iterator &begin,std::list<IMUData>::iterator pback);

}

#endif