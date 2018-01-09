#include "g2otypes.h"

namespace g2o
{

using namespace ORB_SLAM2;

void EdgeNavStateBias::computeError(){
  const VertexNavStateBias* vBiasi = static_cast<const VertexNavStateBias*>(_vertices[0]);
  const VertexNavStateBias* vBiasj = static_cast<const VertexNavStateBias*>(_vertices[1]);
  const NavState& NSi = vBiasi->estimate();const NavState& NSj = vBiasj->estimate();
  
  //rbij/eb=bj-bi see VIORBSLAM paper (6) or Manifold paper (48)
  // residual error of Gyroscope's bias, Forster 15'RSS
  Vector3d rBiasG = (NSj.mbg+NSj.mdbg)-(NSi.mbg+NSi.mdbg);
  // residual error of Accelerometer's bias, Forster 15'RSS
  Vector3d rBiasA = (NSj.mba+NSj.mdba)-(NSi.mba+NSi.mdba);//here is x(x)xa!!!
  Vector6d err(Vector6d::Zero());
  // 6-Dim error vector order: deltabiasGyr_i;deltabiasAcc_i, rBiasGi;rBiasAi
  err.segment<3>(0)=rBiasG;err.segment<3>(3)=rBiasA;
  _error = err;
}
void EdgeNavStateBias::linearizeOplus()
{
  _jacobianOplusXi=-Matrix<double,6,6>::Identity();//J_eb_bi=-I for eb=bj-bi
  _jacobianOplusXj=Matrix<double,6,6>::Identity();//J_eb_bj=I
}
typedef Matrix<double, 15, 1> Vector15d;
void EdgeNavStatePriorPRVBias::computeError()
{
    const VertexNavStatePR* vNSPR=static_cast<const VertexNavStatePR*>(_vertices[0]);//PRj
    const VertexNavStateV* vNSV=static_cast<const VertexNavStateV*>(_vertices[1]);//Vj
    const VertexNavStateBias* vNSBias=static_cast<const VertexNavStateBias*>(_vertices[2]);//Bj
    const NavState& nsPRest=vNSPR->estimate();const NavState& nsVest=vNSV->estimate();const NavState& nsBiasest=vNSBias->estimate();
    
    // P R V bg=bg_bar+dbg ba=ba_bar+dba, see VIORBSLAM paper (8)
    Vector15d err=Vector15d::Zero();
    err.segment<3>(0)=_measurement.mpwb-nsPRest.mpwb;//ep=pwbj_bar-pwbj
    err.segment<3>(3)=(_measurement.mRwb.inverse()*nsPRest.mRwb).log();//eR=Log(Rwbj_bar.t()*Rwbj)
    err.segment<3>(6)=_measurement.mvwb-nsVest.mvwb;//ev=vwbj_bar-vwbj
    err.segment<3>(9)=_measurement.mbg+_measurement.mdbg-(nsBiasest.mbg+nsBiasest.mdbg);//eb=bj_bar-bj
    err.segment<3>(12)=_measurement.mba+_measurement.mdba-(nsBiasest.mba+nsBiasest.mdba);
    _error = err;
}
void EdgeNavStatePriorPRVBias::linearizeOplus()
{
    Matrix<double,15,6> _jacobianOplusPR = Matrix<double,15,6>::Zero();//J_ej_dPRj=[-I Jrinv(phi_eRj)](p<-p+dp); [-Rj ...](p<-p+R*dp)
    
//     const VertexNavStatePR* vNSPR=static_cast<const VertexNavStatePR*>(_vertices[0]);//PRj
//     _jacobianOplusPR.block<3,3>(0,0)= - vNSPR->estimate().getRwb();//p<-p+R*dp
    
    _jacobianOplusPR.block<3,3>(0,0)= - Matrix3d::Identity();//p<-p+dp
    
    _jacobianOplusPR.block<3,3>(3,3)=Sophus::SO3::JacobianRInv( _error.segment<3>(3));//P(R)VBgBa
    Matrix<double,15,3> _jacobianOplusV = Matrix<double,15,3>::Zero();//J_ej_dv=-I
    _jacobianOplusV.block<3,3>(6,0)= - Matrix3d::Identity();//wrong 6 in JingWang code
    Matrix<double,15,6> _jacobianOplusBias = Matrix<double,15,6>::Zero();//j_ej_db=-I
    _jacobianOplusBias.block<3,3>(9,0)= - Matrix3d::Identity();_jacobianOplusBias.block<3,3>(12,3)= - Matrix3d::Identity();

    _jacobianOplus[0] = _jacobianOplusPR;_jacobianOplus[1] = _jacobianOplusV;
    _jacobianOplus[2] = _jacobianOplusBias;
}
void EdgeNavStatePriorPVRBias::computeError()
{
    const VertexNavStatePVR* vNSPVR=static_cast<const VertexNavStatePVR*>(_vertices[0]);//PVRj
    const VertexNavStateBias* vNSBias=static_cast<const VertexNavStateBias*>(_vertices[1]);//Bj
    const NavState& nsPVRest=vNSPVR->estimate();const NavState& nsBiasest=vNSBias->estimate();
    // P V R bg=bg_bar+dbg ba=ba_bar+dba
    Vector15d err=Vector15d::Zero();
    err.segment<3>(0)=_measurement.mpwb-nsPVRest.mpwb;//ep=pwbj_bar-pwbj
    err.segment<3>(6)=(_measurement.mRwb.inverse()*nsPVRest.mRwb).log();//eR=Log(Rwbj_bar.t()*Rwbj)
    err.segment<3>(3)=_measurement.mvwb-nsPVRest.mvwb;//ev=vwbj_bar-vwbj
    err.segment<3>(9)=_measurement.mbg+_measurement.mdbg-(nsBiasest.mbg+nsBiasest.mdbg);//eb=bj_bar-bj
    err.segment<3>(12)=_measurement.mba+_measurement.mdba-(nsBiasest.mba+nsBiasest.mdba);
    _error = err;
}
void EdgeNavStatePriorPVRBias::linearizeOplus()
{
    _jacobianOplusXi= Matrix<double,15,9>::Zero();//J_ej_dPVRj=[-I -I Jrinv(phi_eRj)](p<-p+dp); [-Rj ...](p<-p+R*dp)
    
//     const VertexNavStatePVR* vNSPVR=static_cast<const VertexNavStatePVR*>(_vertices[0]);//PVRj
//     _jacobianOplusXi.block<3,3>(0,0)= - vNSPVR->estimate().getRwb();//p<-p+R*dp
    
    _jacobianOplusXi.block<3,3>(0,0)= - Matrix3d::Identity();//p<-p+dp
    
    _jacobianOplusXi.block<3,3>(3,3)= - Matrix3d::Identity();
    _jacobianOplusXi.block<3,3>(6,6)=Sophus::SO3::JacobianRInv( _error.segment<3>(6));//PV(R)BgBa
    _jacobianOplusXj = Matrix<double,15,6>::Zero();//J_ej_db=-I
    _jacobianOplusXj.block<3,3>(9,0)= - Matrix3d::Identity();_jacobianOplusXj.block<3,3>(12,3)= - Matrix3d::Identity();
}

void EdgeEnc::computeError()
{
  const VertexSE3Expmap* vi=static_cast<const VertexSE3Expmap*>(_vertices[0]);
  const VertexSE3Expmap* vj=static_cast<const VertexSE3Expmap*>(_vertices[1]);
  Quaterniond qRiw=vi->estimate().rotation(),qRjw=vj->estimate().rotation();
  Vector3d piw=vi->estimate().translation(),pjw=vj->estimate().translation();
  Vector3d rEij;
  Sophus::SO3 so3Roioj=Sophus::SO3(qRco.conjugate()*qRiw*qRjw.conjugate()*qRco);
  rEij[0]=so3Roioj.log()[2]-_measurement[0];//(Log(Roc*Rciw*Rwcj*Rco)-delta~phiij).z
//   rEij[0]=Sophus::SO3::log(Sophus::SO3::exp(Vector3d(0,0,_measurement[0])).inverse()*so3Roioj)[2];//Log(delta~Rij.t()*Roioj)
  Vector3d deltapij=qRco.conjugate()*(piw-qRiw*qRjw.conjugate()*pjw-pco+qRiw*qRjw.conjugate()*pco);//Roc*[pciw-Rciw*Rcjw.t()*pcjw-pco+Rciw*Rcjw.t()*pco]
  rEij.segment<2>(1)=deltapij.segment<2>(0)-_measurement.segment<2>(1);//deltapij-delta~pij
  _error=rEij;
}

void EdgeEnc::linearizeOplus()
{
  const VertexSE3Expmap* vi=static_cast<const VertexSE3Expmap*>(_vertices[0]);
  const VertexSE3Expmap* vj=static_cast<const VertexSE3Expmap*>(_vertices[1]);
  Quaterniond qRiw=vi->estimate().rotation(),qRjw=vj->estimate().rotation();
  Vector3d piw=vi->estimate().translation(),pjw=vj->estimate().translation();
  
  //calculate Je_drhoi/j
  Matrix<double,1,3> O1x3=Matrix<double,1,3>::Zero();//JeR_drhoi/j=0
  Matrix3d Jep_drhoi,Jep_drhoj;
  Matrix3d Roc=qRco.conjugate().toRotationMatrix();
  Jep_drhoi=Roc*Sophus::SO3::JacobianL(Sophus::SO3(qRiw).log());//Jep_drhoi=Roc*Jl(phi_iw)
  Quaterniond qRij=qRiw*qRjw.conjugate();
  Quaterniond qRocRij=qRco.conjugate()*qRij;
  Jep_drhoj=-(qRocRij).toRotationMatrix()*Sophus::SO3::JacobianL(Sophus::SO3(qRjw).log());
  
  //calculate Je_dphi_i/j
  Matrix3d Jep_dphii,Jep_dphij;
  Jep_dphii=Roc*Sophus::SO3::hat(qRij*(pjw-pco));//Jep_dphii=Roc*[Riw*Rwj*(pjw-pco)]^
  Jep_dphij=qRocRij.toRotationMatrix()*Sophus::SO3::hat(pco-pjw);//Roc*Riw*Rwj*(pco-pjw)^
  Matrix3d JeR_dphii,JeR_dphij;
  Vector3d eRpart=Sophus::SO3(qRocRij*qRco).log();
  JeR_dphii=Sophus::SO3::JacobianLInv(eRpart)*Roc;
  JeR_dphij=-Sophus::SO3::JacobianRInv(eRpart)*Roc;
//   Sophus::SO3 deltaRjiM=Sophus::SO3::exp(Vector3d(0,0,_measurement[0])).inverse();
//   Vector3d eR=Sophus::SO3(deltaRjiM.unit_quaternion()*qRocRij*qRco).log();
//   JeR_dphii=Sophus::SO3::JacobianRInv(eR)*Roc*qRij.conjugate().toRotationMatrix();
//   JeR_dphij=-Sophus::SO3::JacobianRInv(eR)*Roc;
  
  _jacobianOplusXi.block<1,3>(0,0)=O1x3;
  _jacobianOplusXi.block<2,3>(1,0)=Jep_drhoi.block<2,3>(0,0);
  _jacobianOplusXi.block<1,3>(0,3)=JeR_dphii.block<1,3>(2,0);
  _jacobianOplusXi.block<2,3>(1,3)=Jep_dphii.block<2,3>(0,0);
  _jacobianOplusXj.block<1,3>(0,0)=O1x3;
  _jacobianOplusXj.block<2,3>(1,0)=Jep_drhoj.block<2,3>(0,0);
  _jacobianOplusXj.block<1,3>(0,3)=JeR_dphij.block<1,3>(2,0);
  _jacobianOplusXj.block<2,3>(1,3)=Jep_dphij.block<2,3>(0,0);
}

}