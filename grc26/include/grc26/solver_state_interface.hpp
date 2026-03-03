#ifndef SOLVER_STATE_INTERFACE_HPP
#define SOLVER_STATE_INTERFACE_HPP

#include "grc26/achd_solver.hpp"

class SolverStateInterface
{
public:
  SolverStateInterface(VereshchaginSolver& solver)
    : solver_(solver)
  {}

  inline void toSolver(const SystemState& state)
  {
    for (unsigned int i = 0; i < solver_.q().rows(); ++i)
    {
      solver_.q()(i)   = state.arm.q[i];
      solver_.qd()(i)  = state.arm.qd[i];
    }
  }

  inline void fromSolver(SystemState& state) const
  {
    for (unsigned int i = 0; i < solver_.tauCmd().rows(); ++i)
    {
      state.arm.tau_cmd[i] = solver_.tauCmd()(i);
    }
  }

private:
  VereshchaginSolver& solver_;
};

#endif // SOLVER_STATE_INTERFACE_HPP