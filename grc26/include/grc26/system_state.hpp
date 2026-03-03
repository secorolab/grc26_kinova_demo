#ifndef SYSTEM_STATE_HPP
#define SYSTEM_STATE_HPP

#include <array>
#include <optional>
#include <cstdint>
#include <chrono>
#include "robif2b/functions/kinova_gen3.h"
#include "robif2b/functions/robotiq_ft_sensor.h"
#include "kdl/frames.hpp"

constexpr unsigned int ARM_DOF = 7;

struct SystemState
{
  struct TimeData
  {
    std::chrono::steady_clock::time_point cycle_start;
    std::chrono::steady_clock::time_point cycle_end;
    uint64_t sequence = 0;
    double cycle_time_expected = 0.001;
    double cycle_time_measured = 0.0;
  } time_data;

  struct ArmData
  {
    double cycle_time = 0.001;

    double q[ARM_DOF]         = {0.0};
    double q_cmd[ARM_DOF]     = {0.0};

    double qd[ARM_DOF]        = {0.0};
    double qd_cmd[ARM_DOF]    = {0.0};

    double qdd[ARM_DOF]       = {0.0};

    double tau_msr[ARM_DOF]   = {0.0};
    double tau_cmd[ARM_DOF]   = {0.0};

    double cur_msr[ARM_DOF]   = {0.0};
    double cur_cmd[ARM_DOF]   = {0.0};

    double imu_ang_vel_msr[3] = {0.0};
    double imu_lin_acc_msr[3] = {0.0};

    KDL::Frame pose_ee_BL;
    KDL::Twist twist_ee_BL;

    bool success              = false;
    bool present              = true;
    enum robif2b_ctrl_mode ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
  } arm;

  struct GripperData
  {
    float pos_msr[1] = { 0.0 };
    float vel_msr[1] = { 0.0 };
    float cur_msr[1] = { 0.0 };
    float pos_cmd[1] = { 0.0 };
    float vel_cmd[1] = { 0.0 };
    float frc_cmd[1] = { 0.0 };
    bool success = false;
    bool present = false;
  } gripper;

  struct FTData
  {
    float fx, fy, fz, tx, ty, tz = 0.0;
    float wrench[6];
    bool new_data = false;
    bool success = false;
    enum robif2b_robotiq_ft_state ft_state;
    bool present = true;
  } ft_sensor;
};

#endif // SYSTEM_STATE_HPP