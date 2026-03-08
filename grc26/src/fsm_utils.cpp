#include "grc26/fsm_interface.hpp"

#include <cmath>

void FSMInterface::reset_ft_force_estimator()
{
  ft_reference_ready_ = false;
  ft_reference_count_ = 0;
  ft_reference_sum_.fill(0.0);
  ft_reference_mean_.fill(0.0);
  ft_window_sum_.fill(0.0);
  ft_window_count_ = 0;
  ft_window_index_ = 0;

  for (auto& sample : ft_window_samples_) {
    sample.fill(0.0);
  }
}

void FSMInterface::transform_ft_readings_to_BL_update_state(SystemState& system_state,
  ArmKinematics& arm_kinematics)
{
  for (int axis = 0; axis < 6; ++axis) {
    if (!std::isfinite(system_state.ft_sensor.wrench[axis])) {
      return;
    }
  }

  KDL::Wrench ext_wrench_FT_link_ = KDL::Wrench(
    KDL::Vector(
      static_cast<double>(system_state.ft_sensor.wrench[0]),
      static_cast<double>(system_state.ft_sensor.wrench[1]),
      static_cast<double>(system_state.ft_sensor.wrench[2])
    ),
    KDL::Vector(
      static_cast<double>(system_state.ft_sensor.wrench[3]),
      static_cast<double>(system_state.ft_sensor.wrench[4]),
      static_cast<double>(system_state.ft_sensor.wrench[5])
    )
  );

  const KDL::Frame& ft_link_in_base = arm_kinematics.pose();
  KDL::Wrench ext_wrench_BL = ft_link_in_base * ext_wrench_FT_link_;

  system_state.ft_sensor.fx_BL = static_cast<float>(ext_wrench_BL.force.x());
  system_state.ft_sensor.fy_BL = static_cast<float>(ext_wrench_BL.force.y());
  system_state.ft_sensor.fz_BL = static_cast<float>(ext_wrench_BL.force.z());
  system_state.ft_sensor.tx_BL = static_cast<float>(ext_wrench_BL.torque.x());
  system_state.ft_sensor.ty_BL = static_cast<float>(ext_wrench_BL.torque.y());
  system_state.ft_sensor.tz_BL = static_cast<float>(ext_wrench_BL.torque.z());

  system_state.ft_sensor.wrench_BL[0] = system_state.ft_sensor.fx_BL;
  system_state.ft_sensor.wrench_BL[1] = system_state.ft_sensor.fy_BL;
  system_state.ft_sensor.wrench_BL[2] = system_state.ft_sensor.fz_BL;
  system_state.ft_sensor.wrench_BL[3] = system_state.ft_sensor.tx_BL;
  system_state.ft_sensor.wrench_BL[4] = system_state.ft_sensor.ty_BL;
  system_state.ft_sensor.wrench_BL[5] = system_state.ft_sensor.tz_BL;
}

bool FSMInterface::update_ft_force_estimate(const SystemState& system_state,
                                            std::array<double, 6>& corrected_force_mean)
{
  corrected_force_mean.fill(0.0);

  if (!system_state.ft_sensor.present) {
    return false;
  }

  std::array<double, 6> raw_wrench{};
  for (int axis = 0; axis < 6; ++axis) {
    raw_wrench[axis] = static_cast<double>(system_state.ft_sensor.wrench_BL[axis]);
    if (!std::isfinite(raw_wrench[axis])) {
      if (true) {
        printf("Skipping non-finite FT sample on axis %d\n", axis);
      }
      return false;
    }
  }

  if (!ft_reference_ready_) {
    for (int axis = 0; axis < 6; ++axis) {
      ft_reference_sum_[axis] += raw_wrench[axis];
    }

    ++ft_reference_count_;
    if (ft_reference_count_ >= FT_WINDOW_SIZE) {
      for (int axis = 0; axis < 6; ++axis) {
        ft_reference_mean_[axis] = ft_reference_sum_[axis] / static_cast<double>(FT_WINDOW_SIZE);
      }
      ft_reference_ready_ = true;
      ft_window_sum_.fill(0.0);
      ft_window_count_ = 0;
      ft_window_index_ = 0;
      for (auto& sample : ft_window_samples_) {
        sample.fill(0.0);
      }
      if (true) {
        printf("FT reference captured from %zu samples\n", ft_reference_count_);
      }
    }
    return false;
  }

  if (ft_window_count_ == FT_WINDOW_SIZE) {
    const auto& oldest = ft_window_samples_[ft_window_index_];
    for (int axis = 0; axis < 6; ++axis) {
      ft_window_sum_[axis] -= oldest[axis];
    }
  } else {
    ++ft_window_count_;
  }

  ft_window_samples_[ft_window_index_] = raw_wrench;
  for (int axis = 0; axis < 6; ++axis) {
    ft_window_sum_[axis] += raw_wrench[axis];
  }

  ft_window_index_ = (ft_window_index_ + 1) % FT_WINDOW_SIZE;

  if (ft_window_count_ < FT_WINDOW_SIZE) {
    return false;
  }

  for (int axis = 0; axis < 6; ++axis) {
    const double rolling_mean = ft_window_sum_[axis] / static_cast<double>(FT_WINDOW_SIZE);
    corrected_force_mean[axis] = rolling_mean - ft_reference_mean_[axis];
  }

  return true;
}

void FSMInterface::normalize_angle_diff(double& angle_diff)
{
  if (angle_diff > M_PI) {
    angle_diff -= 2 * M_PI;
  }
  else if (angle_diff < -M_PI) {
    angle_diff += 2 * M_PI;
  }
}

void FSMInterface::avoid_joint_limits(SystemState& system_state)
{
  double stiffness = 150.0; // Nm/rad

  // normalize joint angles at indices 3,5 to be within [-180,180]
  for (int joint_index : {3, 5}) {
    double angle_deg = system_state.arm.q[joint_index];
    if (angle_deg > M_PI) {
      system_state.arm.q[joint_index] -= 2 * M_PI;
    }
    else if (angle_deg < -M_PI) {
      system_state.arm.q[joint_index] += 2 * M_PI;
    }
  }

  double jnt_3_angle_ul_rad = JOINT_3_ANGLE_LIMIT_DEG_UL * M_PI / 180.0;
  double jnt_3_angle_ll_rad = JOINT_3_ANGLE_LIMIT_DEG_LL * M_PI / 180.0;
  double jnt_5_angle_ll_rad = JOINT_5_ANGLE_LIMIT_DEG_LL * M_PI / 180.0;
  double jnt_5_angle_ul_rad = JOINT_5_ANGLE_LIMIT_DEG_UL * M_PI / 180.0;

  if (system_state.arm.q[3] > jnt_3_angle_ul_rad) {
    system_state.arm.tau_cmd[3] = stiffness * (jnt_3_angle_ul_rad - system_state.arm.q[3]);
  }
  else if (system_state.arm.q[3] < jnt_3_angle_ll_rad) {
    system_state.arm.tau_cmd[3] = stiffness * (jnt_3_angle_ll_rad - system_state.arm.q[3]);
  }
  if (system_state.arm.q[5] > jnt_5_angle_ul_rad) {
    system_state.arm.tau_cmd[5] = stiffness * (jnt_5_angle_ul_rad - system_state.arm.q[5]);
  }
  else if (system_state.arm.q[5] < jnt_5_angle_ll_rad) {
    system_state.arm.tau_cmd[5] = stiffness * (jnt_5_angle_ll_rad - system_state.arm.q[5]);
  }
}

void FSMInterface::check_post_condition(events *eventData, const SystemState& system_state, const TaskSpec& task_spec)
{
  if (!task_spec.post_condition.available)
    return;

  bool condition_met = (task_spec.post_condition.logic == LogicOp::And) ? true : false;

  for (int i = 0; i < task_spec.post_condition.num_constraints; ++i)
  {
    const Constraint& constraint = task_spec.post_condition.constraints[i];
    double value = 0.0;

    switch (constraint.type)
    {
      case ConstraintType::Position:
        if (constraint.axis < 3)
          value = arm_kinematics_->pose().p[constraint.axis];
        else if (constraint.axis == 3)
          value = system_state.gripper.pos_msr[0];
        break;

      case ConstraintType::Velocity:
        if (constraint.axis < 3)
          value = arm_kinematics_->twist().vel[constraint.axis];
        else if (constraint.axis == 3)
          value = system_state.gripper.vel_msr[0];
        break;

      case ConstraintType::Force:
        if (constraint.axis < 3)
          value = system_state.ft_sensor.wrench[constraint.axis];
        break;

      case ConstraintType::Torque:
        if (constraint.axis < 3)
          value = system_state.ft_sensor.wrench[constraint.axis + 3];
        break;

      default:
        break;
    }

    bool constraint_satisfied = false;
    switch (constraint.op)
    {
      case CompareOp::LessEqual:
        constraint_satisfied = (value <= constraint.value);
        break;
      case CompareOp::GreaterEqual:
        constraint_satisfied = (value >= constraint.value);
        break;
      case CompareOp::EqualWithinTolerance:
        constraint_satisfied = (std::abs(value - constraint.value) <= constraint.tolerance);
        break;
      default:
        break;
    }

    if (task_spec.post_condition.logic == LogicOp::And)
      condition_met &= constraint_satisfied;
    else
      condition_met |= constraint_satisfied;
  }

  if (condition_met)
  {
    printf("Post condition met\n");
    if (fsm_execution_state == S_M_TOUCH_TABLE){
      produce_event(eventData, E_ENTER_IDLE);
      // produce_event(eventData, E_M_SLIDE_ALONG_TABLE_CONFIG);
      printf("Completed touch table behavior\n");
    }
    else if (fsm_execution_state == S_M_SLIDE_ALONG_TABLE){
      printf("Completed slide along table behavior\n");
      if (system_state.gripper.present) {
          produce_event(eventData, E_M_GRASP_OBJECT_CONFIG);
        }
      else {
        produce_event(eventData, E_ENTER_IDLE);
      }
    }
    else if (fsm_execution_state == S_M_GRASP_OBJECT){
      printf("Completed grasp object behavior\n");
      if (system_state.gripper.present) {
        produce_event(eventData, E_ENTER_IDLE);
      }
    }
    else if (fsm_execution_state == S_M_COLLABORATE){
      printf("Completed collaborate behavior\n");
      if (system_state.gripper.present) {
          produce_event(eventData, E_M_RELEASE_OBJECT_CONFIG);
        }
      else {
        produce_event(eventData, E_ENTER_IDLE);
      }
    }
    else if (fsm_execution_state == S_M_RELEASE_OBJECT){
      printf("Completed release object behavior\n");
      produce_event(eventData, E_ENTER_IDLE);
    }
  }
}