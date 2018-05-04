#pragma once

#include <QKeyEvent>
#include <QGLViewer/qglviewer.h>

#include <srrg_types/cloud_3d.h>

#include <icp_solver/solver.h>

class SolverViewer: public QGLViewer {
public:

  //! ctor
  SolverViewer(QWidget* parent = 0);

  //! init method, opens the gl viewport and sets the key bindings
  void init();

  //! callback invoked by the application on new key event. It saves the last event in
  //! a member variable
  virtual void keyPressEvent(QKeyEvent *e);

  //! returns the last key pressed since invoking keyEventProcessed();
  QKeyEvent* lastKeyEvent();

  //! call this to clear the events, after processing them
  void keyEventProcessed();

  //! draw method
  virtual void draw();

  //!pass scenes to draw
  inline void setScenes(srrg_core::Cloud3D *scene_fixed_,srrg_core::Cloud3D *scene_moving_){
    scene_fixed  = scene_fixed_;
    scene_moving = scene_moving_;
  }

  //! pass solver
  inline void setSolver(Solver *solver_){solver = solver_;}

protected:

  QKeyEvent _last_key_event;
  bool _last_key_event_processed;

  Solver *solver;

  srrg_core::Cloud3D *scene_fixed;
  srrg_core::Cloud3D *scene_moving;

};
