// This file is part of Sophus.
//
// Copyright 2011 Hauke Strasdat (Imperial College London)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights  to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include <iostream>
#include "so3.h"

//ToDo: Think completely through when to normalize Quaternion

namespace Sophus
{

// right jacobian of SO(3)
Matrix3d SO3::JacobianR(const Vector3d& w)
{
    Matrix3d Jr = Matrix3d::Identity();
    double theta = w.norm();
    if(theta < SMALL_EPS)
    {
        Matrix3d Omega = SO3::hat(w);
        Matrix3d Omega2 = Omega * Omega;
        // omit 3rd order eps & more for 1e-5 (accuracy:e-10), similar to omit >=1st order (Jl=I/R) for 1e-10
        return Jr - 0.5 * Omega + Omega2/ 6.;//the one more order term is theta*theta*Omega/24. < 2^(-52), where 1 + it is useless
    }
    else
    {
        Vector3d k = w.normalized();  // k - unit direction vector of w
        Matrix3d K = SO3::hat(k);
        Jr =   Matrix3d::Identity()
                - (1-cos(theta))/theta*K
                + (1-sin(theta)/theta)*K*K;
    }
    return Jr;
}
Matrix3d SO3::JacobianRInv(const Vector3d& w)
{
    Matrix3d Jrinv = Matrix3d::Identity();
    double theta = w.norm();
    Matrix3d Omega = SO3::hat(w);

    // very small angle
    if(theta < SMALL_EPS)
    {
        //limit(theta->0)((1-theta/(2*tan(theta/2)))/theta^2)~=(omit theta^5&&less)=1/12
        return Jrinv + 0.5 * Omega + (1./12.)*(Omega*Omega);
    }
    else
    {
        Vector3d k = w.normalized();  // k - unit direction vector of w
        Matrix3d K = SO3::hat(k);
        Jrinv = Matrix3d::Identity()
                + 0.5 * Omega
                + ( 1.0 - (1.0+cos(theta))*theta / (2.0*sin(theta)) ) *K*K;
    }

    return Jrinv;
    /*
 * in gtsam:
 *
 *   double theta2 = omega.dot(omega);
 *  if (theta2 <= std::numeric_limits<double>::epsilon()) return I_3x3;
 *  double theta = std::sqrt(theta2);  // rotation angle
 *  * Right Jacobian for Log map in SO(3) - equation (10.86) and following equations in
 *   * G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups", Volume 2, 2008.
 *   * logmap( Rhat * expmap(omega) ) \approx logmap( Rhat ) + Jrinv * omega
 *   * where Jrinv = LogmapDerivative(omega);
 *   * This maps a perturbation on the manifold (expmap(omega))
 *   * to a perturbation in the tangent space (Jrinv * omega)
 *
 *  const Matrix3 W = skewSymmetric(omega); // element of Lie algebra so(3): W = omega^
 *  return I_3x3 + 0.5 * W +
 *         (1 / (theta * theta) - (1 + cos(theta)) / (2 * theta * sin(theta))) *
 *             W * W;
 *
 * */
}

// left jacobian of SO(3), Jl(x) = Jr(-x)
Matrix3d SO3::JacobianL(const Vector3d& w)
{
    return JacobianR(-w);
}
// left jacobian inverse
Matrix3d SO3::JacobianLInv(const Vector3d& w)
{
    return JacobianRInv(-w);
}
// ---------------------------------

SO3::SO3()
{
  unit_quaternion_.setIdentity();
}

SO3
::SO3(const SO3 & other) : unit_quaternion_(other.unit_quaternion_)
{
    unit_quaternion_.normalize();
}

SO3
::SO3(const Matrix3d & R) : unit_quaternion_(R)
{
    unit_quaternion_.normalize();
}

SO3
::SO3(const Quaterniond & quat) : unit_quaternion_(quat)
{
  assert(unit_quaternion_.squaredNorm() > SMALL_EPS);
  unit_quaternion_.normalize();
}

SO3
::SO3(double rot_x, double rot_y, double rot_z)
{
  unit_quaternion_
      = (SO3::exp(Vector3d(rot_x, 0.f, 0.f))
         *SO3::exp(Vector3d(0.f, rot_y, 0.f))
         *SO3::exp(Vector3d(0.f, 0.f, rot_z))).unit_quaternion_;
}

void SO3
::operator=(const SO3 & other)
{
  this->unit_quaternion_ = other.unit_quaternion_;
}

SO3 SO3
::operator*(const SO3& other) const
{
  SO3 result(*this);
  result.unit_quaternion_ *= other.unit_quaternion_;
  result.unit_quaternion_.normalize();
  return result;
}

void SO3
::operator*=(const SO3& other)
{
  unit_quaternion_ *= other.unit_quaternion_;
  unit_quaternion_.normalize();
}

Vector3d SO3
::operator*(const Vector3d & xyz) const
{
  return unit_quaternion_._transformVector(xyz);
}

SO3 SO3
::inverse() const
{
  return SO3(unit_quaternion_.conjugate());
}

Matrix3d SO3
::matrix() const
{
  return unit_quaternion_.toRotationMatrix();
}

Matrix3d SO3
::Adj() const
{
  return matrix();//Ad(R)=R
}

Matrix3d SO3
::generator(int i)//generate one of the hat of 3 axis unit vector
{
  assert(i>=0 && i<3);
  Vector3d e;
  e.setZero();
  e[i] = 1.f;
  return hat(e);
}

Vector3d SO3
::log() const
{
  return SO3::log(*this);
}

Vector3d SO3
::log(const SO3 & other)
{
  double theta;
  return logAndTheta(other, &theta);
}

//range[-pi,pi) is mainly designed for residual error
Vector3d SO3
::logAndTheta(const SO3 & other, double * theta)
{

    double n = other.unit_quaternion_.vec().norm();//|sin(theta/2)|
    double w = other.unit_quaternion_.w();//cos(theta/2)
    double squared_w = w*w;

    double two_atan_nbyw_by_n;
    // Atan-based log thanks to
    //
    // C. Hertzberg et al.:
    // "Integrating Generic Sensor Fusion Algorithms with Sound State
    // Representation through Encapsulation of Manifolds"
    // Information Fusion, 2011

    // small variable approximation is used for speed but keep the max or reasonable accuracy
    // (so3.cpp here choose the max or double(1) + double(2^(-52)))
    if (n < SMALL_EPS)
    {
      // If quaternion is normalized and n=1, then w should be 1;
      // w=0 should never happen here!
      assert(fabs(w)>SMALL_EPS);

      two_atan_nbyw_by_n = 2./w - 2./3*(n*n)/(w*squared_w);
    }
    else
    {
      if (fabs(w)<SMALL_EPS)
      {//notice atan(x) = pi/2 - atan(1/x)
        if (w>0)
        {//notice for range[-pi,pi), atan(x) = pi/2 - atan(1/x) for x>0
          two_atan_nbyw_by_n = M_PI/n;
        }
        else//w=0 corresponds to theta= Pi or -Pi, here choose -Pi
        {//notice for range[-pi,pi), atan(x) = -pi/2 - atan(1/x) for x<=0
          two_atan_nbyw_by_n = -M_PI/n;//theta belongs to [-Pi,Pi)=>theta/2 in [-Pi/2,Pi/2)
        }
        double n_pow2 = n * n;
        double n_pow4 = n_pow2 * n_pow2;
        two_atan_nbyw_by_n -= 2 * w / n_pow2 - 2. / 3 * (w * squared_w) / n_pow4;
      }else
	two_atan_nbyw_by_n = 2*atan(n/w)/n;//theta/sin(theta/2)
    }

    *theta = two_atan_nbyw_by_n*n;
    return two_atan_nbyw_by_n * other.unit_quaternion_.vec();
}

SO3 SO3
::exp(const Vector3d & omega)
{
  double theta;
  return expAndTheta(omega, &theta);
}

SO3 SO3
::expAndTheta(const Vector3d & omega, double * theta)
{
  *theta = omega.norm();
  double half_theta = 0.5*(*theta);

  double imag_factor;
  double real_factor;
  if((*theta) < SMALL_EPS)
  {
    double theta_sq = (*theta)*(*theta);
//    double theta_po4 = theta_sq*theta_sq;
    imag_factor = 0.5 - theta_sq / 48.;// + theta_po4 / 3840.;//Taylor expansion of sin(x/2)/x
    real_factor = 1.0 - theta_sq / 8.;// + theta_po4 / 384.;
    //real_factor = cos(half_theta);
  }
  else
  {
    double sin_half_theta = sin(half_theta);
    imag_factor = sin_half_theta/(*theta);
    real_factor = cos(half_theta);
  }

  return SO3(Quaterniond(real_factor,
                         imag_factor*omega.x(),
                         imag_factor*omega.y(),
                         imag_factor*omega.z()));
}

Matrix3d SO3
::hat(const Vector3d & v)
{
  Matrix3d Omega;
  Omega <<  0, -v(2),  v(1)
      ,  v(2),     0, -v(0)
      , -v(1),  v(0),     0;
  return Omega;
}

Vector3d SO3
::vee(const Matrix3d & Omega)//will judge/assert if it's skewSymmetric
{
  assert(fabs(Omega(2,1)+Omega(1,2))<SMALL_EPS);
  assert(fabs(Omega(0,2)+Omega(2,0))<SMALL_EPS);
  assert(fabs(Omega(1,0)+Omega(0,1))<SMALL_EPS);
  return Vector3d(Omega(2,1), Omega(0,2), Omega(1,0));
}

Vector3d SO3
::lieBracket(const Vector3d & omega1, const Vector3d & omega2)
{
  return omega1.cross(omega2);//lieBracket for Vector3d is cross, as g=[R^3,R,X]
}

Matrix3d SO3
::d_lieBracketab_by_d_a(const Vector3d & b)
{
  return -hat(b);//par(Exp(phi)p/b)/par(phi).t()=J_Rp_phi=-(Rp)^=-b^
}

void SO3::
setQuaternion(const Quaterniond& quaternion)
{
  assert(quaternion.norm()!=0);
  unit_quaternion_ = quaternion;
  unit_quaternion_.normalize();
}


}
