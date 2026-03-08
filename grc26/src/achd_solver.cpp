#include "grc26/achd_solver.hpp"
#include <iostream>

VereshchaginSolver::VereshchaginSolver(const ArmKDLModel& model)
    : model_(model)
{
    for (unsigned int i = 0; i < tau_cmd_.rows(); ++i)
    {
      tau_cmd_(i) = 0.0;
      tau_cmd_fext_(i) = 0.0;
    }
}

void VereshchaginSolver::setAlpha(const KDL::Jacobian& alpha)
{
    if (alpha.columns() != num_constraints_)
        throw std::runtime_error("Alpha dimension mismatch");

    alpha_ = alpha;
}

void VereshchaginSolver::setState(const SystemState& state)
{
  for (unsigned int i = 0; i < dof_; ++i)
  {
    q_(i)  = state.arm.q[i];
    qd_(i) = state.arm.qd[i];
  }
}

void VereshchaginSolver::updateTorqueCmdInState(SystemState& state) const
{
  for (unsigned int i = 0; i < tau_cmd_.rows(); ++i)
  {
    const double main_tau = std::isfinite(tau_cmd_(i)) ? tau_cmd_(i) : 0.0;
    const double fext_tau = std::isfinite(tau_cmd_fext_(i)) ? tau_cmd_fext_(i) : 0.0;
    state.arm.tau_cmd[i] = static_cast<float>(main_tau + fext_tau);
  }
}

void VereshchaginSolver::updateTorqueCmdFromRNEAInState(SystemState& state) const
{
  for (unsigned int i = 0; i < tau_cmd_rnea.rows(); ++i)
  {
    const double rnea_tau = std::isfinite(tau_cmd_rnea(i)) ? tau_cmd_rnea(i) : 0.0;
    state.arm.tau_cmd[i] = static_cast<float>(rnea_tau);
  }
}

void VereshchaginSolver::resetTorqueOutputs() { 
    for (unsigned int i = 0; i < tau_cmd_.rows(); ++i)
    {
      tau_cmd_(i) = 0.0;
    }
    for (unsigned int i = 0; i < tau_cmd_fext_.rows(); ++i)
    {
      tau_cmd_fext_(i) = 0.0;
    }
    for (unsigned int i = 0; i < tau_cmd_rnea.rows(); ++i)
    {
      tau_cmd_rnea(i) = 0.0;
    }
  }

bool VereshchaginSolver::initialize(unsigned int num_constraints)
{
  num_constraints_ = num_constraints;
  dof_             = model_.num_joints();
  num_segments_    = model_.num_segments();

  if (dof_ == 0)
  {
    std::cerr << "Solver init failed: model not loaded\n";
    return false;
  }

  if (num_segments_ == 0)
  {
    std::cerr << "Solver init failed: model has zero segments\n";
    return false;
  }

  // allocate solver state variables
  q_            = KDL::JntArray(dof_);
  qd_           = KDL::JntArray(dof_);
  qdd_          = KDL::JntArray(dof_);
  qdd_fext_     = KDL::JntArray(dof_);
  jnt_accelerations = KDL::JntArray(dof_);
  tau_cmd_      = KDL::JntArray(dof_);
  tau_cmd_fext_ = KDL::JntArray(dof_);
  tau_cmd_rnea  = KDL::JntArray(dof_);
  ff_taus_      = KDL::JntArray(dof_);
  ff_taus_fext_ = KDL::JntArray(dof_);
  beta_         = KDL::JntArray(num_constraints_);
  beta_fext_    = KDL::JntArray(num_constraints_);

  alpha_      = KDL::Jacobian(num_constraints_);
  alpha_fext_ = KDL::Jacobian(num_constraints_);
  f_ext_      = KDL::Wrenches(num_segments_);
  f_ext_fext_ = KDL::Wrenches(num_segments_);

  // gravity vector from model
  KDL::Vector gravity_vec = model_.gravity();

  // Gravity twist. As solver expects gravity to be in the negative direction
  KDL::Twist g_twist(
    KDL::Vector(-gravity_vec.x(), -gravity_vec.y(), -gravity_vec.z()+0.5),
    KDL::Vector::Zero()
  );

  // Initialize a zero gravity vector for fext solver
  KDL::Twist g_twist_zero(KDL::Vector::Zero(), KDL::Vector::Zero());

  // Initialise RNEA solver
  jacobDotSolver = std::make_unique<KDL::ChainJntToJacDotSolver>(model_.chain());
  ikSolverAcc = std::make_unique<KDL::ChainIkSolverVel_pinv>(model_.chain());

  KDL::Vector gravity(gravity_vec.x(),
                      gravity_vec.y(),
                      gravity_vec.z());

  idSolver = std::make_unique<KDL::ChainIdSolver_RNE>(model_.chain(), gravity);


  // End of RNEA solver initialization

  // Initialize Vereshchagin solvers
  solver_fixed_jnt_ = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(
          model_.chain(), g_twist, num_constraints_);

  solver_fext_ = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(
          model_.chain(), g_twist_zero, num_constraints_);
  // set zero external forces
  for (size_t i = 0; i < f_ext_.size(); ++i) {
    f_ext_[i] = KDL::Wrench::Zero();
    f_ext_fext_[i] = KDL::Wrench::Zero();
  }

  alpha_.setColumn(0, KDL::Twist(KDL::Vector(1, 0, 0), KDL::Vector(0, 0, 0)));
  alpha_.setColumn(1, KDL::Twist(KDL::Vector(0, 1, 0), KDL::Vector(0, 0, 0))); 
  alpha_.setColumn(2, KDL::Twist(KDL::Vector(0, 0, 1), KDL::Vector(0, 0, 0))); 
  alpha_.setColumn(3, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(1, 0, 0))); 
  alpha_.setColumn(4, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0, 1, 0))); 
  alpha_.setColumn(5, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 1)));

  alpha_fext_.setColumn(0, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0)));
  alpha_fext_.setColumn(1, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0))); 
  alpha_fext_.setColumn(2, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0))); 
  alpha_fext_.setColumn(3, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0))); 
  alpha_fext_.setColumn(4, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0))); 
  alpha_fext_.setColumn(5, KDL::Twist(KDL::Vector(0, 0, 0), KDL::Vector(0, 0, 0)));

  initialized_ = true;
  return true;
}

int VereshchaginSolver::computeTorques()
{
  if (!initialized_ || !solver_fixed_jnt_)
    return -1;

  return solver_fixed_jnt_->CartToJnt(q_,
                                      qd_,
                                      qdd_,
                                      alpha_,
                                      beta_,
                                      f_ext_,
                                      ff_taus_,
                                      tau_cmd_);
}

int VereshchaginSolver::computeTorquesFext()
{
  if (!initialized_ || !solver_fext_)
    return -1;

  return solver_fext_->CartToJnt(q_,
                                 qd_,
                                 qdd_fext_,
                                 alpha_fext_,
                                 beta_fext_,
                                 f_ext_fext_,
                                 ff_taus_fext_,
                                 tau_cmd_fext_);
}

void VereshchaginSolver::computeTorquesRNEA(
    KDL::JntArrayVel &jnt_velocities,
    KDL::JntArray &jnt_positions,
    KDL::JntArray &jnt_velocity,
    KDL::Wrenches &linkWrenches)
{
  jacobDotSolver->JntToJacDot(jnt_velocities, jd_qd);
  xdd_minus_jd_qd = xdd - jd_qd;
  ikSolverAcc->CartToJnt(jnt_positions, xdd_minus_jd_qd, jnt_accelerations);
  idSolver->CartToJnt(jnt_positions, jnt_velocity, jnt_accelerations, linkWrenches, tau_cmd_rnea);
}