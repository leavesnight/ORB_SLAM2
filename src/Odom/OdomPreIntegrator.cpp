//created by zzh
#include "OdomPreIntegrator.h"
#include "IMUInitialization.h"//for color cout

using namespace std;

namespace ORB_SLAM2{

using namespace Eigen;

double EncPreIntegrator::msigma2Model=1e-4;
void EncPreIntegrator::PreIntegration(const double &timeStampi,const double &timeStampj,
				      const listeig(EncData)::const_iterator &iterBegin,const listeig(EncData)::const_iterator &iterEnd){
  if (iterBegin!=iterEnd){ 
    Vector2d eigdeltaPijM(0,0);//deltaPii=0
    double deltaThetaijMz=0;//deltaTheta~iiz=0
    mSigmaEij.setZero();//SigmaEii=0
    
    Matrix2d eigSigmaetad(EncData::mSigmad);double rc(EncData::mrc);
    
    for (listeig(EncData)::const_iterator iterj=iterBegin;iterj!=iterEnd;){//start iterative method from i/iteri->tm to j/iter->tm
      listeig(EncData)::const_iterator iterjm1=iterj++;//iterj-1
      
      double deltat,tj,tj_1;//deltatj-1j
      if (iterjm1==iterBegin) tj_1=timeStampi; else tj_1=iterjm1->mtm;
      if (iterj==iterEnd) tj=timeStampj; else tj=iterj->mtm;
      deltat=tj-tj_1;
      assert(deltat>=0);
      if (deltat>1.5){ mdeltatij=0;cout<<redSTR"Check Odometry!"<<whiteSTR<<endl;return;}//this filter is for my dataset's problem
      
      //selete/design measurement_j-1
      double vl,vr;//vlj-1,vrj-1
      if (iterj!=iterEnd){
	vl=(iterjm1->mv[0]+iterj->mv[0])/2,vr=(iterjm1->mv[1]+iterj->mv[1])/2;
      }else{
	vl=iterjm1->mv[0];vr=iterjm1->mv[1];
      }
      double vf=(vl+vr)/2,w=(-vl+vr)/2/rc;//[vf;w]k=1/2*[1 1;-1/rc 1/rc]*[vl;vr]k, here k=j-1
      
      //calculate Sigmaij firstly to use deltaThetaijMz as deltaTheta~ij-1z, maybe we can use deltaP~ij-1 instead of vf/w here
      double arrdTmp2[9]={1,sin(deltaThetaijMz)*vf*deltat,-cos(deltaThetaijMz)*vf*deltat, 0,1,0, 0,0,1};
      double arrdTmp3[3]={deltat/rc/2,cos(deltaThetaijMz)*deltat/2,sin(deltaThetaijMz)*deltat/2};
      double arrdTmp4[6]={arrdTmp3[0],arrdTmp3[1],arrdTmp3[2], -arrdTmp3[0],arrdTmp3[1],arrdTmp3[2]};
      Matrix3d A(arrdTmp2);Matrix<double,3,2> B(arrdTmp4);
      mSigmaEij=A*mSigmaEij*A.transpose()+B*eigSigmaetad*B.transpose();
      
      //update deltaPijM before update deltaThetaijM to use deltaThetaijMz as deltaTheta~ij-1z
      double arrdTmp[4]={cos(deltaThetaijMz),sin(deltaThetaijMz),-sin(deltaThetaijMz),cos(deltaThetaijMz)};//row-major:{cos(deltaTheij-1Mz),-sin(deltaTheij-1Mz),0,sin(deltaTheij-1Mz),cos(deltaThetaij-1Mz),0,0,0,1}; but Eigen defaultly uses col-major!
      eigdeltaPijM+=Eigen::Matrix2d(arrdTmp)*Eigen::Vector2d(vf*deltat,0);//deltaPijM+Roioj-1*voj-1oj-1*deltat
      //update deltaThetaijM
      deltaThetaijMz+=w*deltat;//deltaThetaij-1M + woioj-1 * deltatj-1j
    }
    
    mdelxEij[0]=deltaThetaijMz;mdelxEij.segment<2>(1)=eigdeltaPijM;
    mdeltatij=timeStampj-timeStampi;
  }
}
void IMUPreIntegratorDerived::PreIntegration(const double &timeStampi,const double &timeStampj){
  if (!this->mlOdom.empty()){
    IMUDataDerived& datai=this->mlOdom.front(),dataj=this->mlOdom.back();
    mdelxRji=dataj.quat.conjugate()*datai.quat;//R~j.t()*R~i, suppose quat is normalized~
    mSigmaPhiij=IMUDataDerived::mSigmaI;
    Matrix3d Ai(mdelxRji*datai.getJacoright());
    //get SigmaIij=Ai*Sigmaetawi*Ai.t()+Jrj*Sigmaetawi*Jrj.t(),Ai=(R~j.t()*R~i)*Jr(phiwIb(ti))=(R~j.t()*R~i)*Jr(R~wibi)
    Eigen::Matrix3d Jrj=dataj.getJacoright();
    mSigmaPhiij=Ai*mSigmaPhiij*Ai.transpose()+Jrj*mSigmaPhiij*Jrj.transpose();
    this->mdeltatij=timeStampj-timeStampi;
  }
}

}