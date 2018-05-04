#include <iostream>
#include <icp_solver/solver.h>

using namespace srrg_core;

int main(){

  //world points
  size_t n=100;
  Vector3fVector scene_moving(n);
  for(Eigen::Vector3f &l : scene_moving)
    l = (Eigen::Vector3f::Random() - 0.5*Eigen::Vector3f::Ones())*10;

  //transform
  Vector6f v;
  v << 0.1, 0.1, 0.1, 0.0f, 0.0f, 0.0f;
  Eigen::Isometry3f T = srrg_core::v2tEuler(v);

  //scene_fixed
  Vector3fVector scene_fixed(n);
  Solver::ConstraintVector constraints(n);
  for(size_t i=0; i<n; ++i){
    scene_fixed[i] = T*scene_moving[i];
    constraints[i] = Solver::Constraint(scene_fixed[i],scene_moving[i]);
  }

  //solver
  Solver solver;
  solver.constraints() = constraints;

  //perturb initial guess
  Vector6f pv;
  pv << -0.1, 0.0, 0.0, 0.0f, 0.0f, 0.0f;
  Eigen::Isometry3f pT = srrg_core::v2tEuler(pv);
  solver.init(T*pT);

  //run ICP
  size_t iterations=10;
  for(size_t i=0; i<iterations; ++i){
    std::cerr << "Iteration: " << i+1 << std::endl;
    solver.oneRound();
    std::cerr << "solving..." << std::endl;
    std::cerr << "T:" << std::endl;
    std::cerr << solver.transform().translation().transpose() << " ";
    std::cerr << solver.transform().linear().eulerAngles(0,1,2).transpose() << std::endl;
    std::cerr << "Chi_square: " << solver.chiSquare() << std::endl << std::endl;

    solver.init(solver.transform());
  }

  return 0;
}
