#include <iostream>
#include <icp_solver/solver.h>
#include <icp_solver_viewer/solver_viewer.h>

#include <qapplication.h>
#include <qglviewer.h>

using namespace srrg_core;

int main(int argc, char** argv){

  //world points
  size_t n=100;
  Cloud3D *scene_moving = new Cloud3D();
  scene_moving->resize(n);
  for(size_t i=0; i<n; ++i){
    const Eigen::Vector3f p = (Eigen::Vector3f::Random() - 0.5*Eigen::Vector3f::Ones())*10;
    scene_moving->at(i) = RichPoint3D(p,Eigen::Vector3f::Zero(),1.0f,Eigen::Vector3f(1,0,0));
  }

  //transform
  Vector6f v;
  v << 0.1, 0.1, 0.1, 0.0f, 0.0f, 0.0f;
  Eigen::Isometry3f T = srrg_core::v2tEuler(v);

  //scene_fixed
  Cloud3D *scene_fixed = new Cloud3D();
  scene_fixed->resize(n);
  Solver::ConstraintVector constraints(n);
  for(size_t i=0; i<n; ++i){
    const Eigen::Vector3f p = T*scene_moving->at(i).point();
    scene_fixed->at(i) = RichPoint3D(p,Eigen::Vector3f::Zero(),1.0f,Eigen::Vector3f(0,0,1));
    constraints[i] = Solver::Constraint(p,scene_moving->at(i).point());
  }

  //solver
  Solver solver;
  solver.constraints() = constraints;

  //viewer
  QApplication app(argc, argv);
  SolverViewer viewer;
  viewer.setScenes(scene_fixed,scene_moving);
  viewer.show();

  //perturb initial guess
  Vector6f pv;
  pv << -0.1, 0.0, 0.0, 0.0f, 0.0f, 0.0f;
  Eigen::Isometry3f pT = srrg_core::v2tEuler(pv);
  solver.init(T*pT);

  bool continue_=true;
  while(continue_){

    viewer.updateGL();
    app.processEvents();

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

  }


  app.exec();

  delete scene_fixed;
  delete scene_moving;

  return 0;
}
