#ifndef SYSTEM_STATE_HPP
#define SYSTEM_STATE_HPP

#include <array>
#include <optional>
#include <cstdint>
#include <chrono>
#include <atomic>
#include "robif2b/functions/kinova_gen3.h"
#include "robif2b/functions/robotiq_ft_sensor.h"

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
    double cycle_time         = 0.001;

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
    bool to_control_gripper = false;
    bool is_gripper_moving = false;
    bool gripper_control_completed = false;
    bool present = false;
    static constexpr const char* kComPort = "/dev/ttyUSB0";
    static constexpr int kBaudRate = 115200;
    static constexpr double kTimeout = 1.0;
    static constexpr int kSlaveAddress = 0x09;
  } gripper;

  struct FTData
  {
    float fx, fy, fz, tx, ty, tz = 0.0;
    float fx_BL, fy_BL, fz_BL, tx_BL, ty_BL, tz_BL = 0.0;
    float wrench[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float wrench_BL[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    bool new_data = false;
    bool success = false;
    enum robif2b_robotiq_ft_state ft_state;
    bool present = true;
  } ft_sensor;
};


struct FTSnapshot
{
  float fx = 0.0f;
  float fy = 0.0f;
  float fz = 0.0f;
  float tx = 0.0f;
  float ty = 0.0f;
  float tz = 0.0f;

  float wrench[6]{};

  bool new_data = false;
  bool success = false;

  robif2b_robotiq_ft_state ft_state = ROBIF2B_ROBOTIQ_FT_STATE_INIT;

  std::chrono::steady_clock::time_point sample_time = std::chrono::steady_clock::now();

  std::uint64_t sequence = 0;
};

struct FTIOBuffer
{
  float fx = 0.0f;
  float fy = 0.0f;
  float fz = 0.0f;
  float tx = 0.0f;
  float ty = 0.0f;
  float tz = 0.0f;

  float wrench[6]{};

  bool new_data = false;
  bool success = false;

  robif2b_robotiq_ft_state ft_state = ROBIF2B_ROBOTIQ_FT_STATE_INIT;
};

struct GripperIOBuffer
{
  std::atomic<float> pos_cmd{0.0f};
  std::atomic<float> pos_msr{0.0f};
  std::atomic<float> vel_msr{0.0f};
  std::atomic<float> cur_msr{0.0f};
  std::atomic<bool> to_control_gripper{false};
  std::atomic<bool> is_gripper_moving{false};
  std::atomic<bool> gripper_control_completed{false};
  std::atomic<bool> success{false};
};
#endif // SYSTEM_STATE_HPP