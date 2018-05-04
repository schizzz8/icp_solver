add_library(icp_solver_library SHARED
  solver.cpp solver.h
)

target_link_libraries(icp_solver_library
  ${catkin_LIBRARIES}
)
