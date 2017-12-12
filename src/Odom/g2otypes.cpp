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
    Matrix<double,15,6> _jacobianOplusPR = Matrix<double,15,6>::Zero();//J_ej_dPRj=[-I Jrinv(phi_eRj)]
    _jacobianOplusPR.block<3,3>(0,0)= - Matrix3d::Identity();
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
    _jacobianOplusXi= Matrix<double,15,9>::Zero();//J_ej_dPVRj=[-I -I Jrinv(phi_eRj)]
    _jacobianOplusXi.block<3,3>(0,0)= - Matrix3d::Identity();
    _jacobianOplusXi.block<3,3>(3,3)= - Matrix3d::Identity();
    _jacobianOplusXi.block<3,3>(6,6)=Sophus::SO3::JacobianRInv( _error.segment<3>(6));//PV(R)BgBa
    _jacobianOplusXj = Matrix<double,15,6>::Zero();//j_ej_db=-I
    _jacobianOplusXj.block<3,3>(9,0)= - Matrix3d::Identity();_jacobianOplusXj.block<3,3>(12,3)= - Matrix3d::Identity();
}

}