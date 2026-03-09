#include "grc26/compute_controller_command.hpp"

ComputeControllerCommand::ComputeControllerCommand(const Controllers& controllers)
: controllers_(controllers)
{
}

void ComputeControllerCommand::compute(
    const SystemState& state,
    const ArmKinematics& kin,
    const TaskSpec& task,
    KDL::JntArray& beta,
    KDL::Wrenches& f_ext,
    double dt,
    DebugSample* debug_sample)
{
    if (dt <= 0.0)
        dt = (state.arm.cycle_time > 0.0) ? state.arm.cycle_time : 1e-3;

    // ---- reset outputs ----
    for (unsigned int i = 0; i < beta.rows(); ++i)
        beta(i) = 0.0;

    for (size_t i = 0; i < f_ext.size(); ++i)
        f_ext[i] = KDL::Wrench::Zero();

    const KDL::Vector& ee_pos   = kin.pose().p;
    const KDL::Twist&  ee_twist = kin.twist();

    if (debug_sample) {
        for (int axis = 0; axis < 3; ++axis) {
            debug_sample->ee_vel[axis] = ee_twist.vel[axis];
            debug_sample->ee_vel[3 + axis] = ee_twist.rot[axis];
            debug_sample->ee_vel_error[axis] = 0.0;
            debug_sample->ee_vel_error[3 + axis] = 0.0;
            debug_sample->control_signal[axis] = 0.0;
            debug_sample->control_signal[3 + axis] = 0.0;
            debug_sample->pid_axes[axis] = PIDAxisDebug{};
            debug_sample->pid_axes[3 + axis] = PIDAxisDebug{};
        }
    }

    const auto capture_pid_axis = [&](int axis, const PID& pid, double error, double control_sig) {
        if (!debug_sample || axis < 0 || axis >= 6) {
            return;
        }
        debug_sample->ee_vel_error[axis] = error;
        debug_sample->control_signal[axis] = control_sig;
        debug_sample->pid_axes[axis].p = pid.last_p_term;
        debug_sample->pid_axes[axis].i = pid.last_i_term;
        debug_sample->pid_axes[axis].d = pid.last_d_term;
        debug_sample->pid_axes[axis].error = pid.last_error;
        debug_sample->pid_axes[axis].control_sig = pid.last_output;
    };

    // ---- Linear Control ----
    if (task.ee_linear.enabled)
    {
        for (int i = 0; i < 3; ++i)
        {
            switch (task.ee_linear.mode[i])
            {
            case LinearMode::Velocity:
            {
                double err = task.ee_linear.velocity[i] - ee_twist.vel[i];
                double ctrl_out = controllers_.cart_ctrl[i].control(err, dt);
                beta(i) = 0.0;
                f_ext.back()(i) = - ctrl_out; // ensure no external force in linear axes for velocity control
                capture_pid_axis(i, controllers_.cart_ctrl[i], err, ctrl_out);
                break;
            }

            case LinearMode::Position:
            {
                printf("[WARNING!!] Position control in linear motion is not fully tested, use with caution!!\n");
                double pos_error = task.ee_linear.position[i] - ee_pos[i];
                // clamp pos_error with vref
                pos_error = std::max(std::min(pos_error, task.ee_linear.vel_threshold), -task.ee_linear.vel_threshold);

                beta(i) = controllers_.cart_ctrl[i].control(pos_error, dt);
                capture_pid_axis(i, controllers_.cart_ctrl[i], pos_error, beta(i));
                if (i == 2)
                {
                    printf("Position error in z: %f, velocity: %f, beta: %f\n", pos_error, ee_twist.vel[i], beta(i));
                }
                else
                {
                    printf("Position error in axis %d: %f, velocity: %f, beta: %f\n", i, pos_error, ee_twist.vel[i], beta(i));
                }
                break;
            }

            case LinearMode::Force:
            {
                f_ext.back()(i) = - task.ee_linear.force[i];
                break;
            }

            case LinearMode::None:
            {
                break;
            }

            default:
                break;
            }
        }
    }

    // ---- Orientation Control ----
    if (task.orientation.enabled)
    {
        switch (task.orientation.mode)
        {
        case OrientationMode::Position:
        {
            KDL::Rotation desired_ee_rot_ = KDL::Rotation::RPY(
                                            task.orientation.rpy[0],
                                            task.orientation.rpy[1],
                                            task.orientation.rpy[2]);
            KDL::Vector diff = KDL::diff(kin.pose().M,desired_ee_rot_);

            int seg = task.orientation.segment_index;

            // printf("[Current orientation] RPY: [%f, %f, %f]\n", kin.rpy()[0], kin.rpy()[1], kin.rpy()[2]);
            // printf("[Desired orientation] the desired rotation matrix: \n");
            // printf("%f, %f, %f\n", 
            //     desired_ee_rot_(0,0), desired_ee_rot_(0,1), desired_ee_rot_(0,2));
            // printf("%f, %f, %f\n", 
            //     desired_ee_rot_(1,0), desired_ee_rot_(1,1), desired_ee_rot_(1,2));
            // printf("%f, %f, %f\n", 
            //     desired_ee_rot_(2,0), desired_ee_rot_(2,1), desired_ee_rot_(2,2));

            printf("[Orientation control]: RPY error [%f, %f, %f]\n", diff(0), diff(1), diff(2));

            for (int i = 0; i < 3; ++i){
                const double rot_err = diff(i);
                if (seg == 8) // if controlling at the end-effector, using beta
                {
                    const double pid_output = controllers_.cart_ctrl[i+3].control(rot_err, dt);
                    f_ext[seg](3 + i) = -pid_output;
                    capture_pid_axis(3 + i, controllers_.cart_ctrl[i+3], rot_err, pid_output);
                    printf("[Orientation control]: external torque for axis %d: %f\n", i, pid_output);
                    // beta(3 + i) = controllers_.cart_ctrl[i+3].control(diff(i));
                    // printf("[Orientation control]: beta for axis %d: %f\n", i, beta(3 + i));
                }
                else // if controlling at a different segment, using fext interface to apply torques
                {
                    const double pid_output = controllers_.cart_ctrl[i+3].control(rot_err, dt);
                    f_ext[seg](3 + i) = -pid_output;
                    capture_pid_axis(3 + i, controllers_.cart_ctrl[i+3], rot_err, pid_output);
                }
            }

            // f_ext[seg](3) = 0.0;
            // f_ext[seg](4) = 0.0;
            // f_ext[seg](5) = 0.0;
            break;
        }

        case OrientationMode::Velocity:
        {
            int seg = task.orientation.segment_index;

            for (int i = 0; i < 3; ++i){
                if (seg == 8) // if controlling at the end-effector, using beta
                {
                    const double err = task.orientation.ang_vel[i] - ee_twist.rot[i];
                    beta(3 + i) = controllers_.cart_ctrl[i+3].control(err, dt);
                    capture_pid_axis(3 + i, controllers_.cart_ctrl[i+3], err, beta(3 + i));
                }
                else // if controlling at a different segment, we use the velocity error to compute desired angular velocity for damping control
                {
                    const double err = task.orientation.ang_vel[i] - ee_twist.rot[i];
                    const double pid_output = controllers_.cart_ctrl[i+3].control(err, dt);
                    f_ext[seg](3 + i) = -pid_output;
                    capture_pid_axis(3 + i, controllers_.cart_ctrl[i+3], err, pid_output);
                }
            }

            break;
        }

        case OrientationMode::Torque:
        {
            int seg = task.orientation.segment_index;

            for (int i = 0; i < 3; ++i)
                f_ext[seg](3 + i) = -task.orientation.torque[i];

            break;
        }

        case OrientationMode::None:
        {
            break;
        }

        default:
            break;
        }
    }

    // -------- Additional Link Forces --------
    if (task.link_force.enabled)
    {
        int seg = task.link_force.segment_index;
        for (int i = 0; i < 3; ++i)
        {
            f_ext[seg](i) = - task.link_force.force[i];
        }
    }

    // By default, control the forearm to have same yaw throughout the task
    if (task.forearm_yaw_control_enabled)
    {
        double forearm_link_y_axis_angle_sp = 0.0;
        double stiffness_forearm_y_axis_angle = 35.0;
        double deadband_forearm_y_axis_angle = 0.0;
        double torque_limit_forearm_link = 25.0;

        KDL::Vector torque_vector = KDL::Vector::Zero();

        // get y-axis of link3 (forearm) in arm base_link frame
        KDL::Vector measured_ForeArm_Link_y_axis_BL = kin.forearmPoseBL().M.UnitY();

        if (false) printf("Forearm pose: position [%f, %f, %f]\n", kin.forearmPoseBL().p.x(), kin.forearmPoseBL().p.y(), kin.forearmPoseBL().p.z());

        // project y-axis onto xy-plane of arm base_link to get angle made with the plane
        KDL::Vector measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane = KDL::Vector(measured_ForeArm_Link_y_axis_BL.x(), measured_ForeArm_Link_y_axis_BL.y(), 0.0);
        measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane = measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane / measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane.Norm();
        double angle = std::acos(KDL::dot(measured_ForeArm_Link_y_axis_BL, measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane));

        // get torque axis as the cross product between y-axis and its projection vector. This will change direction when the forearm y-axis goes above or below the xy-plane
        KDL::Vector torque_axis = measured_ForeArm_Link_y_axis_BL * measured_ForeArm_Link_y_axis_BL_projection_to_XY_plane;
        torque_axis = torque_axis / torque_axis.Norm();

        double error = 0;
        if (angle < (forearm_link_y_axis_angle_sp - deadband_forearm_y_axis_angle)) {
        error = (forearm_link_y_axis_angle_sp - deadband_forearm_y_axis_angle) - angle;
        } else if (angle > (forearm_link_y_axis_angle_sp + deadband_forearm_y_axis_angle)) {
        error = (forearm_link_y_axis_angle_sp + deadband_forearm_y_axis_angle) - angle;
        } else {
        error = 0;
        }
        double torque_magnitude = std::abs(stiffness_forearm_y_axis_angle * error);
        if (torque_magnitude > torque_limit_forearm_link) {
        torque_magnitude = torque_limit_forearm_link;
        } else if (torque_magnitude < -torque_limit_forearm_link) {
        torque_magnitude = -torque_limit_forearm_link;
        }

        // transform torque axis to forearm link frame (not used)
        KDL::Vector torque_axis_forearm_link = kin.forearmPoseBL().M.Inverse() * torque_axis;

        // achd solver takes wrenches in base link frame
        torque_vector = torque_magnitude * torque_axis;

        if (false) printf("Forearm yaw control: angle %f, error %f, torque magnitude %f, torque axis in BL [%f, %f, %f], torque vector in BL [%f, %f, %f]\n",
        angle, error, torque_magnitude,
        torque_axis.x(), torque_axis.y(), torque_axis.z(),
        torque_vector.x(), torque_vector.y(), torque_vector.z());

        int forearm_seg = 3; // forearm link segment index. Also hard-coded in arm_kinematics.cpp when computing forearm pose
        f_ext[forearm_seg-1](3) = - torque_vector.x();
        f_ext[forearm_seg-1](4) = - torque_vector.y();
        f_ext[forearm_seg-1](5) = - torque_vector.z();
    }
}
