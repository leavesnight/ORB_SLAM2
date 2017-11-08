//created by zzh
#include "OdomPreIntegrator.h"
#include "Converter.h"

using namespace std;

namespace ORB_SLAM2{

double OdomPreIntegrator::mrc=1;
cv::Mat OdomPreIntegrator::mSigmad(2,2,CV_32F),OdomPreIntegrator::mSigmaI(3,3,CV_32F);

template <>//specialized definition must be defined only once/put in .cpp file
void OdomPreIntegrator::SetPreIntegrationList<IMUData>(std::list<IMUData>::iterator &begin,std::list<IMUData>::iterator pback){
  mlOdomIMU.clear();
  mlOdomIMU.push_front(*begin);
  mlOdomIMU.push_back(*pback);
}
void OdomPreIntegrator::PreIntegration(const double timeStampi,const double timeStamp){
  if (!mlOdomEnc.empty()){ 
    Eigen::Vector2d eigdeltaPijM(0,0);//deltaPii=0
    double deltaThetaijMz=0;//deltaTheta~iiz=0
    Eigen::Matrix3d eigSigmaEij(Eigen::Matrix3d::Zero());//SigmaEii=0
    Eigen::Matrix2d eigSigmaetad(Converter::toMatrix2d(mSigmad));
    list<EncData>::iterator iterEnd=mlOdomEnc.end();
    for (list<EncData>::iterator iterj=mlOdomEnc.begin();iterj!=iterEnd;){//start iterative method from i/iteri->tm to j/iter->tm
      list<EncData>::iterator iterjm1=iterj++;//iterj-1
      double deltat,tj,tj_1;//deltatj-1j
      if (iterjm1==mlOdomEnc.begin()) tj_1=timeStampi; else tj_1=iterjm1->tm;
      if (iterj==mlOdomEnc.end()) tj=timeStamp; else tj=iterj->tm;
      deltat=tj-tj_1;
      double vl,vr;//vlj-1,vrj-1
      if (iterj!=mlOdomEnc.end()){
	vl=(iterjm1->v[0]+iterj->v[0])/2,vr=(iterjm1->v[1]+iterj->v[1])/2;
      }else{
	vl=iterjm1->v[0];vr=iterjm1->v[1];
      }
      double vf=(vl+vr)/2,w=(-vl+vr)/2/mrc;//[vf;w]=1/2*[1 1;-1/rc 1/rc]*[vl;vr]
      double deltaThetaz=deltaThetaijMz;//record deltaTheta~ij-1z
      //update deltaThetaijM
      deltaThetaijMz+=w*deltat;//deltaThetaij-1M + woioj-1 * deltatj-1j
      //update deltaPijM
      double arrdTmp[4]={cos(deltaThetaz),sin(deltaThetaz),-sin(deltaThetaz),cos(deltaThetaz)};//row-major:{cos(deltaTheij-1Mz),-sin(deltaTheij-1Mz),0,sin(deltaTheij-1Mz),cos(deltaThetaij-1Mz),0,0,0,1}; but Eigen defaultly uses col-major!
      eigdeltaPijM+=Eigen::Matrix2d(arrdTmp)*Eigen::Vector2d(vf*deltat,0);//deltaPijM+Roioj-1*voj-1oj-1*deltat
      
      double arrdTmp2[9]={1,sin(deltaThetaz)*vf*deltat,-cos(deltaThetaz)*vf*deltat, 0,1,0, 0,0,1};
      double arrdTmp3[3]={deltat/mrc/2,cos(deltaThetaz)*deltat/2,sin(deltaThetaz)*deltat/2};
      double arrdTmp4[6]={arrdTmp3[0],arrdTmp3[1],arrdTmp3[2], -arrdTmp3[0],arrdTmp3[1],arrdTmp3[2]};
      Eigen::Matrix3d A(arrdTmp2);Eigen::Matrix<double,3,2> B(arrdTmp4);
      eigSigmaEij=A*eigSigmaEij*A.transpose()+B*eigSigmaetad*B.transpose();
    }
    mdelxEij=Converter::toCvMat(Eigen::Vector3d(deltaThetaijMz,eigdeltaPijM[0],eigdeltaPijM[1]));
    mSigmaEij=Converter::toCvMat(eigSigmaEij);
  }
  if (!mlOdomIMU.empty()){
    IMUData& datai=mlOdomIMU.front(),dataj=mlOdomIMU.back();
    Eigen::Quaterniond eigqji=Eigen::Quaterniond(dataj.quat).conjugate()*Eigen::Quaterniond(datai.quat);//R~j.t()*R~i
    Eigen::Matrix3d eigSigmaqIij(Converter::toMatrix3d(mSigmaI)),Ai(Eigen::Matrix3d(eigqji)*datai.getJacoright());
    //get SigmaIij=Ai*Sigmaetawi*Ai.t()+Jrj*Sigmaetawi*Jrj.t(),Ai=(R~j.t()*R~i)*Jr(phiwIb(ti))=(R~j.t()*R~i)*Jr(R~wibi)
    Eigen::Matrix3d Jrj=dataj.getJacoright();
    eigSigmaqIij=Ai*eigSigmaqIij*Ai.transpose()+Jrj*eigSigmaqIij*Jrj.transpose();
    mdelxqIijT=Converter::toQuat(eigqji);
    mSigmaqIij=Converter::toCvMat(eigSigmaqIij);
  }
}

}