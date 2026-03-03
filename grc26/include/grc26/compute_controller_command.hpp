#ifndef COMPUTE_CONTROLLER_COMMAND_HPP
#define COMPUTE_CONTROLLER_COMMAND_HPP

#include "grc26/task_setpoint.hpp"
#include "grc26/pid_controller.hpp"
#include "grc26/stiffness_controller.hpp"
#include "grc26/system_state.hpp"

constexpr int MAX_LINKS = ARM_DOF + 1;

struct ControlOutput
{
    double ee_acceleration[6];        // beta
    double f_ext[MAX_LINKS][6];       // wrench injection
};

void compute_controller_command(
    const SystemState&        state,
    const TaskSetpoint&       sp,
    ControlOutput&            out,
    PID                       pid_lin[3],   // x, y, z linear PID controllers
    const StiffnessController ori_ctrl[3],  // roll, pitch, yaw stiffness controllers
    double                    dt = 0.001);

#endif // COMPUTE_CONTROLLER_COMMAND_HPP  