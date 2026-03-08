#ifndef ACHD_SOLVER_HPP
#define ACHD_SOLVER_HPP

#include <memory>
#include "kdl/chainhdsolver_vereshchagin_fixed_joint.hpp"
#include "kdl/chainhdsolver_vereshchagin_fext_fixed_joint.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/frames.hpp>

#include "grc26/arm_kdl_model.hpp"
#include "grc26/system_state.hpp"


class VereshchaginSolver
{
public:
  explicit VereshchaginSolver(const ArmKDLModel& model);

  bool initialize(unsigned int num_constraints = 6);

  void setAlpha(const KDL::Jacobian& alpha);

  void setState(const SystemState& state);

  void updateTorqueCmdInState(SystemState& state) const;

  void updateTorqueCmdFromRNEAInState(SystemState& state) const;

  KDL::Wrenches& externalWrenches() noexcept { return f_ext_; }
  const KDL::Wrenches& externalWrenches() const noexcept { return f_ext_; }

  KDL::Wrenches& externalWrenches_fext_solver() noexcept { return f_ext_fext_; }
  const KDL::Wrenches& externalWrenches_fext_solver() const noexcept { return f_ext_fext_; }
  /*
  Usage:
  auto& f_ext_rt = solver.externalWrenches();
  f_ext_rt.back() = measured_wrench; // apply wrench at end-effector segment
  OR
  f_ext_rt[segment_index] = wrench; // apply wrench at specified segment
  */

  KDL::JntArray& beta() { return beta_; }
  const KDL::JntArray& beta() const { return beta_; }

  int computeTorques();
  int computeTorquesFext();

  void computeTorquesRNEA(
    KDL::JntArrayVel &jnt_velocities,
    KDL::JntArray &jnt_positions,
    KDL::JntArray &jnt_velocity,
    KDL::Wrenches &linkWrenches);

  const KDL::JntArray& qdd() const { return qdd_; }
  const KDL::JntArray& tauCmd() const { return tau_cmd_; }
  const KDL::JntArray& tauCmdFext() const { return tau_cmd_fext_; }
  const KDL::JntArray& tauCmdRNEA() const { return tau_cmd_rnea; }
  void resetTorqueOutputs();


private:
  const ArmKDLModel& model_;

  std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint> solver_fixed_jnt_;
  std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint> solver_fext_;
  std::unique_ptr<KDL::ChainJntToJacDotSolver> jacobDotSolver;
  std::unique_ptr<KDL::ChainIkSolverVel_pinv> ikSolverAcc;
  std::unique_ptr<KDL::ChainIdSolver_RNE> idSolver;


  unsigned int dof_{0};
  unsigned int num_segments_{0};
  unsigned int num_constraints_{6};

  // initialization of variables for solver inputs and outputs
  KDL::JntArray q_;
  KDL::JntArray qd_;
  KDL::JntArray qdd_;
  KDL::JntArray qdd_fext_;

  KDL::JntArray tau_cmd_;
  KDL::JntArray tau_cmd_fext_;
  KDL::JntArray ff_taus_;
  KDL::JntArray ff_taus_fext_;
  KDL::JntArray beta_;
  KDL::JntArray beta_fext_;

  KDL::Jacobian alpha_;
  KDL::Jacobian alpha_fext_;
  KDL::Wrenches f_ext_;
  KDL::Wrenches f_ext_fext_;

  // RNEA solver variables
  KDL::Twist xdd;
  KDL::Twist xdd_minus_jd_qd;
  KDL::Twist jd_qd;
  KDL::JntArray jnt_accelerations;
  KDL::JntArray tau_cmd_rnea;

  bool initialized_{false};
};

#endif // ACHD_SOLVER_HPP