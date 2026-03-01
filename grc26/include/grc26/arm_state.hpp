#ifndef ARM_STATE_HPP
#define ARM_STATE_HPP

#include "robif2b/functions/kinova_gen3.h"

#define NUM_JOINTS 7

// TODO: includce ft-sensor data in arm state
struct ArmState {
    struct {
        struct timespec cycle_start;
        struct timespec cycle_end;
        long cycle_time_msr; // [us]
        long cycle_time_exp; // [us]
    } time;

    struct KinovaArmState {
        bool success;
        enum robif2b_ctrl_mode ctrl_mode;
        double pos_msr[NUM_JOINTS];
        double vel_msr[NUM_JOINTS];
        double eff_msr[NUM_JOINTS];
        double cur_msr[NUM_JOINTS];
        double pos_cmd[NUM_JOINTS];
        double vel_cmd[NUM_JOINTS];
        double eff_cmd[NUM_JOINTS];
        double cur_cmd[NUM_JOINTS];
        double imu_ang_vel_msr[3];
        double imu_lin_acc_msr[3];

        // Gripper fields
        float gripper_pos_msr[1];
        float gripper_vel_msr[1];
        float gripper_cur_msr[1];
        float gripper_pos_cmd[1];
        float gripper_vel_cmd[1];
        float gripper_frc_cmd[1];
    };
    
    KinovaArmState kinova_arm_state;
};

#endif // ARM_STATE_HPP