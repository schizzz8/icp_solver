#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <srrg_types/types.hpp>

class Solver{
public:
  struct Constraint{
    Constraint(const Eigen::Vector3f &fixed_ = Eigen::Vector3f::Zero(),
               const Eigen::Vector3f &moving_ = Eigen::Vector3f::Zero()):
      fixed(fixed_),moving(moving_){}

    Eigen::Vector3f fixed;
    Eigen::Vector3f moving;
  };
  typedef std::vector<Constraint> ConstraintVector;

  Solver();

  void init(const Eigen::Isometry3f &T_ = Eigen::Isometry3f::Identity());

  void oneRound();

  inline const ConstraintVector& constraints() const { return _constraints; }
  inline ConstraintVector& constraints()  {return _constraints;}

  inline const Eigen::Isometry3f& transform() const {return _T;}
  inline void setTransform(const Eigen::Isometry3f& transform_=Eigen::Isometry3f::Identity()){_T=transform_;}

  inline const float &chiSquare() const {return _chi_square;}

protected:

  void errorAndJacobian(Eigen::Vector3f &e,
                        srrg_core::Matrix3_6f &J,
                        const Eigen::Vector3f &fixed,
                        const Eigen::Vector3f &moving);

  void linearizeConstraint(const Constraint & constraint);

  ConstraintVector _constraints;
  Eigen::Isometry3f _T;

  srrg_core::Matrix6f _H;
  srrg_core::Vector6f _b;

  float _chi_square;

private:
  srrg_core::Vector6f t2v(const Eigen::Isometry3f &t);
  Eigen::Matrix3f Rx_prime(float rot_x);
  Eigen::Matrix3f Ry_prime(float rot_y);
  Eigen::Matrix3f Rz_prime(float rot_z);

};
