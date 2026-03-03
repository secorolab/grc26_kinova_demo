#ifndef ACHD_SOLVER_HPP
#define ACHD_SOLVER_HPP

#include <memory>
#include <kdl/chainfksolvervel_recursive.hpp>
#include "kdl/chainhdsolver_vereshchagin_fixed_joint.hpp"
#include "kdl/chainhdsolver_vereshchagin_fext.hpp"
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>

#include "grc26/arm_kdl_model.hpp"


class VereshchaginSolver
{
public:
  explicit VereshchaginSolver(const ArmKDLModel& model);

  bool initialize(unsigned int num_constraints = 6);

  void setAlpha(const KDL::Jacobian& alpha);

  KDL::JntArray& beta() { return beta_; }
  const KDL::JntArray& beta() const { return beta_; }

  int computeTorques();

  KDL::JntArray& q()       { return q_; }
  KDL::JntArray& qd()      { return qd_; }
  const KDL::JntArray& qdd() const { return qdd_; }
  const KDL::JntArray& tauCmd() const { return tau_cmd_; }

private:
  const ArmKDLModel& model_;

  std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint> solver_;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;

  unsigned int dof_{0};
  unsigned int num_segments_{0};
  unsigned int num_constraints_{6};

  // initialization of variables for solver inputs and outputs
  KDL::JntArray q_;
  KDL::JntArray qd_;
  KDL::JntArray qdd_;

  KDL::JntArray tau_cmd_;
  KDL::JntArray ff_taus_;
  KDL::JntArray beta_;

  KDL::Jacobian alpha_;
  KDL::Wrenches f_ext_;

  bool initialized_{false};
};

#endif // ACHD_SOLVER_HPP