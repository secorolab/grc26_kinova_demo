#ifndef ARM_KINEMATICS_HPP
#define ARM_KINEMATICS_HPP

#include <iostream>
#include <array>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include "grc26/system_state.hpp"

class SystemState;

class ArmKinematics
{
public:
  ArmKinematics(const KDL::Chain& chain);

  void update(const SystemState& state) noexcept;

  KDL::JntArray& jointPositions() noexcept { return q_kdl_; }
  const KDL::JntArray& jointPositions() const noexcept { return q_kdl_; }
  KDL::JntArray& jointVelocity() noexcept { return qd_kdl_; }
  const KDL::JntArray& jointVelocity() const noexcept { return qd_kdl_; }
  KDL::JntArrayVel& jointVelocities() noexcept { return q_vel_; }
  const KDL::JntArrayVel& jointVelocities() const noexcept { return q_vel_; }
  const KDL::Frame& pose() const noexcept { return pose_; }
  const KDL::Frame& forearmPoseBL() const noexcept { return forearm_pose_BL_; }
  const KDL::Twist& twist() const noexcept { return twist_; }
  const double* rpy() const noexcept { return rpy_; }

private:
  unsigned int dof_;

  KDL::ChainFkSolverPos_recursive fk_solver_;
  KDL::ChainFkSolverVel_recursive vel_solver_;

  KDL::JntArray q_kdl_;
  KDL::JntArray qd_kdl_;
  KDL::JntArrayVel q_vel_;
  KDL::FrameVel frame_vel_;

  KDL::Frame pose_;
  KDL::Frame forearm_pose_BL_;
  KDL::Twist twist_;
  double rpy_[3];
};

#endif