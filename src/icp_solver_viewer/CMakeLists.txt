add_library(icp_solver_viewer_library SHARED
  solver_viewer.cpp solver_viewer.h
)

target_link_libraries(icp_solver_viewer_library
  ${SRRG_QT_LIBRARIES}
  ${QGLVIEWER_LIBRARY}
  ${catkin_LIBRARIES}
)
