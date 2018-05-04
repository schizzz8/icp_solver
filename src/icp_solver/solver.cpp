#include "solver.h"

using namespace srrg_core;

Solver::Solver(){
  _T.setIdentity();
}

void Solver::init(const Eigen::Isometry3f &T_){

  assert(_constraints.size() && "[Solver][Init]: Invoking solver with no constraints!!!");

  _T=T_;

  _H.setZero();
  _b.setZero();

  _chi_square = 0;
}

void Solver::errorAndJacobian(Eigen::Vector3f &e,
                              Matrix3_6f &J,
                              const Eigen::Vector3f &fixed,
                              const Eigen::Vector3f &moving){

  //compute error
  e = _T*moving - fixed;

  //compute Jacobian
  const Vector6f v = Solver::t2v(_T);
  const Eigen::Matrix3f rx = srrg_core::Rx(v[3]);
  const Eigen::Matrix3f ry = srrg_core::Ry(v[4]);
  const Eigen::Matrix3f rz = srrg_core::Rz(v[5]);
  const Eigen::Matrix3f rx_prime = Solver::Rx_prime(v[3]);
  const Eigen::Matrix3f ry_prime = Solver::Ry_prime(v[4]);
  const Eigen::Matrix3f rz_prime = Solver::Rz_prime(v[5]);

  J.block<3,3>(0,0).setIdentity();
  J.block<3,1>(0,3) = rx_prime*ry*rz*moving;
  J.block<3,1>(0,4) = rx*ry_prime*rz*moving;
  J.block<3,1>(0,5) = rx*ry*rz_prime*moving;

}

void Solver::linearizeConstraint(const Constraint &constraint){
  const Eigen::Vector3f &fixed  = constraint.fixed;
  const Eigen::Vector3f &moving = constraint.moving;

  Eigen::Vector3f e;
  e.setZero();
  Matrix3_6f J;
  J.setZero();

  errorAndJacobian(e,J,fixed,moving);

  _chi_square += e.transpose()*e;

  _H += J.transpose()*J;
  _b += J.transpose()*e;
}

Vector6f Solver::t2v(const Eigen::Isometry3f &t){
  Vector6f v;
  v.head<3>()=t.translation();
  v.tail<3>()=t.linear().eulerAngles(0,1,2);
  return v;
}

void Solver::oneRound(){
  for (Constraint& constraint:_constraints) {
    linearizeConstraint(constraint);
  }
  Vector6f dx=_H.ldlt().solve(-_b);
  _T=srrg_core::v2tEuler(dx)*_T;
}

Eigen::Matrix3f Solver::Rx_prime(float rot_x){
  float c=cos(rot_x);
  float s=sin(rot_x);
  Eigen::Matrix3f R;
  R << 0,  0, 0,
    0,  -s,  -c,
    0,  c,  -s;
  return R;
}

Eigen::Matrix3f Solver::Ry_prime(float rot_y){
  float c=cos(rot_y);
  float s=sin(rot_y);
  Eigen::Matrix3f R;
  R << -s,  0, c,
    0,  0,  0,
    -c,  0,  -s;
  return R;
}

Eigen::Matrix3f Solver::Rz_prime(float rot_z){
  float c=cos(rot_z);
  float s=sin(rot_z);
  Eigen::Matrix3f R;
  R << -s,  -c, 0,
    c,  -s,  0,
    0,  0,  0;
  return R;
}

