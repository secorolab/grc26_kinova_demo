#ifndef COMPUTE_CONTROLLER_COMMAND_HPP
#define COMPUTE_CONTROLLER_COMMAND_HPP


#include "grc26/controller_config.hpp"
#include "grc26/arm_kinematics.hpp"
#include "grc26/system_state.hpp"
#include "grc26/task_spec.hpp"
#include "grc26/achd_solver.hpp"
#include "grc26/controller_config.hpp"
#include "grc26/debug_signals.hpp"

class ComputeControllerCommand
{
public:
    explicit ComputeControllerCommand(const Controllers& controllers);

    void compute(
        const SystemState& state,
        const ArmKinematics& kin,
        const TaskSpec& task,
        KDL::JntArray& beta,
        KDL::Wrenches& f_ext,
        double dt = 0.001,
        DebugSample* debug_sample = nullptr);

    // setGains refreshes the local copy of controllers and its internal states
    void setGains(const Controllers& controllers) { controllers_ = controllers; };

private:
    Controllers controllers_;  // local copy, fixed size
};

#endif // COMPUTE_CONTROLLER_COMMAND_HPP