#ifndef HARDWARE_BINDING_HPP
#define HARDWARE_BINDING_HPP

#include "system_state.hpp"
#include "robif2b/functions/kinova_gen3.h"
#include "robif2b/functions/robotiq_ft_sensor.h"

void bindKinovaArm(
    robif2b_kinova_gen3_nbx& arm,
    SystemState& state)
{
    arm.cycle_time  = &state.arm.cycle_time;
    arm.ctrl_mode   = &state.arm.ctrl_mode;
    arm.jnt_pos_msr = &state.arm.q[0];
    arm.jnt_vel_msr = &state.arm.qd[0];
    arm.jnt_trq_msr = &state.arm.tau_msr[0];
    arm.act_cur_msr = &state.arm.cur_msr[0];
    arm.jnt_pos_cmd = &state.arm.q[0];
    arm.jnt_vel_cmd = &state.arm.qd[0];
    arm.jnt_trq_cmd = &state.arm.tau_cmd[0];
    arm.act_cur_cmd = &state.arm.cur_cmd[0];
    arm.imu_ang_vel_msr = &state.arm.imu_ang_vel_msr[0];
    arm.imu_lin_acc_msr = &state.arm.imu_lin_acc_msr[0];
    arm.success = &state.arm.success;
}

void bindRobotiqGripper(
    robif2b_kg3_robotiq_gripper_nbx& gripper,
    SystemState& state)
{
    gripper.gripper_pos_msr = &state.gripper.pos_msr[0];
    gripper.gripper_vel_msr = &state.gripper.vel_msr[0];
    gripper.gripper_cur_msr = &state.gripper.cur_msr[0];

    gripper.gripper_pos_cmd = &state.gripper.pos_cmd[0];
    gripper.gripper_vel_cmd = &state.gripper.vel_cmd[0];
    gripper.gripper_frc_cmd = &state.gripper.frc_cmd[0];
}

void bindRobotiqFT(
    robif2b_robotiq_ft_nbx& ft,
    SystemState& state)
{
    ft.force_x  = &state.ft_sensor.fx;
    ft.force_y  = &state.ft_sensor.fy;
    ft.force_z  = &state.ft_sensor.fz;
    ft.torque_x = &state.ft_sensor.tx;
    ft.torque_y = &state.ft_sensor.ty;
    ft.torque_z = &state.ft_sensor.tz;
    ft.wrench   = &state.ft_sensor.wrench[0];
    ft.new_data = &state.ft_sensor.new_data;
    ft.state    = &state.ft_sensor.ft_state;
    ft.success  = &state.ft_sensor.success;
}

#endif // HARDWARE_BINDING_HPP