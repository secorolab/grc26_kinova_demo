#include "grc26/fsm_interface.hpp"

#include <cmath>

FSMInterface::FSMInterface(SystemState& system_state,
                           robif2b_kinova_gen3_nbx& rob, 
                           robif2b_kg3_robotiq_gripper_nbx& gripper,
                           robif2b_robotiq_ft_nbx& ft_sensor,
                           TaskStatusData& status)
  : system_state(system_state),
    rob(rob),
    gripper(gripper),
    ft_sensor(ft_sensor),
    task_status(status),
    compute_ctr_cmd_obj(ctr_config_idle.controllers()),
    in_comm_with_hw(false)
{}

FSMInterface::~FSMInterface() {
}

void FSMInterface::configure(events *eventData, SystemState& system_state){

  // Initialize final EE pose for trajectory tracking
  final_ee_pose_ = KDL::Frame(
                    KDL::Rotation::RPY(-1.55, 0.05, -2.80), 
                    KDL::Vector(0.24, -0.56, 0.05));

  // initialise KDL model of the arm from URDF
  model_ = std::make_unique<ArmKDLModel>();
  bool kdl_model_loaded =
    model_->loadFromURDF("grc26",
                    "GEN3_URDF_V12.urdf",
                    "base_link",
                    "FTSensor_Link");

  if (!kdl_model_loaded) {
    printf("Failed to parse URDF file");
    produce_event(eventData, E_CONFIGURE_EXIT);
    return;
  }

  int num_joints = model_->num_joints();
  assert(num_joints == NUM_JOINTS && "Kinova arm has unexpected number of joints");

  // initialise solvers
  solver_ = std::make_unique<VereshchaginSolver>(*model_);
  arm_kinematics_ = std::make_unique<ArmKinematics>(model_->chain());

  if (!solver_->initialize(6)) {
    printf("Solver initialization failed\n");
    produce_event(eventData, E_CONFIGURE_EXIT);
    return;
  }

  // load controller parameters from config file
  ctr_config_idle.load("ctr_gains_idle.yaml");
  ctr_config_touch_table.load("ctr_gains_touch_table.yaml");
  ctr_config_slide_on_table.load("ctr_gains_slide_on_table.yaml");
  ctr_config_grasp_object.load("ctr_gains_grasp_object.yaml");
  ctr_config_collaborate.load("ctr_gains_collaborate.yaml");
  ctr_config_traj_tracking.load("ctr_gains_traj_tracking.yaml");
  ctr_config_release_object.load("ctr_gains_release_object.yaml");

  compute_ctr_cmd_obj.setGains(ctr_config_idle.controllers());
  reset_ft_force_estimator();

  // establish communication with arm
  if (system_state.arm.present){
    robif2b_kinova_gen3_configure(&rob);
    if (!rob.success) {
        printf("Error during gen3_configure\n");
        robif2b_kinova_gen3_shutdown(&rob);
        if (!rob.success) {
            printf("Error during gen3_shutdown\n");
            produce_event(eventData, E_CONFIGURE_EXIT);
            return;
        }
    }

    robif2b_kinova_gen3_recover(&rob);
    if (!rob.success) {
        printf("Error during gen3_recover\n");
        robif2b_kinova_gen3_shutdown(&rob);
        if (!rob.success) {
            printf("Error during gen3_shutdown\n");
            produce_event(eventData, E_CONFIGURE_EXIT);
            return;
        }
    }

    printf("Starting\n");
    robif2b_kinova_gen3_start(&rob);
    if (!rob.success) {
        printf("Error during gen3_start\n");
        robif2b_kinova_gen3_stop(&rob);
        printf("Stopped\n");
    }
  }

  if (system_state.ft_sensor.present) {
    robif2b_robotiq_ft_configure(&ft_sensor);
    if (!ft_sensor.success) {
        printf("Error during ft_sensor configure\n");
        robif2b_robotiq_ft_shutdown(&ft_sensor);
        if (!ft_sensor.success) {
            printf("Error during ft_sensor shutdown\n");
            produce_event(eventData, E_CONFIGURE_EXIT);
            return;
        }
    }
    robif2b_robotiq_ft_start(&ft_sensor);
    if (!ft_sensor.success) {
        printf("Error during ft_sensor start\n");
        robif2b_robotiq_ft_stop(&ft_sensor);
        printf("Stopped ft_sensor\n");
    }
  }

  if (system_state.gripper.present) {
    robif2b_kg3_robotiq_gripper_start(&gripper);
    if (!gripper.success) {
        printf("Error during gripper start\n");
        robif2b_kg3_robotiq_gripper_stop(&gripper);
        printf("Stopped gripper\n");
    }
  }

  in_comm_with_hw = true;

  produce_event(eventData, E_CONFIGURED);
}

void FSMInterface::idle(events *eventData, SystemState& system_state){

  // TODO: this is to test any behavior. Modify events returned in post_condition; an event to start will be triggered by action server
  task_triggered = false;
  if (!task_triggered && task_status.goal_in) 
  {
    task_triggered = true;
    produce_event(eventData, E_M_TOUCH_TABLE_CONFIG);
    // produce_event(eventData, E_M_COLLABORATE_CONFIG);
  }

  // TODO: update task_status accordingly throughout the behaviors
  task_status.idle = true;
  fsm_execution_state = S_IDLE;

  robif2b_kinova_gen3_update(&rob);
  arm_kinematics_->update(system_state);
  if (system_state.ft_sensor.present) {
    transform_ft_readings_to_BL_update_state(system_state, *arm_kinematics_);
  }
  // Optionally, update alpha for solver in all 6 axes
  /* KDL::Jacobian alpha_ = KDL::Jacobian(6);
     alpha_.setColumn(0, KDL::Twist(KDL::Vector(1, 0, 0), KDL::Vector(0, 0, 0)));
     solver_->setAlpha(alpha_);
  */
  compute_ctr_cmd_obj.setGains(ctr_config_idle.controllers());
  task_spec.resetDefault();
  bool use_joint_pos_stiffness_ctrl = true;
  bool use_vel_ctrl = false; // if false, use position control

  if (use_joint_pos_stiffness_ctrl)
  {
    task_spec.joint_position.enabled = true;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      task_spec.joint_position.position[i] = system_state.arm.q[i];
    }
  }
  else if(use_vel_ctrl){
    task_spec.ee_linear.enabled = true;
    task_spec.ee_linear.mode[0] = LinearMode::Velocity;
    task_spec.ee_linear.mode[1] = LinearMode::Velocity;
    task_spec.ee_linear.mode[2] = LinearMode::Velocity;

    task_spec.ee_linear.velocity[0] = 0.0; // m/s
    task_spec.ee_linear.velocity[1] = 0.0; // m/s
    task_spec.ee_linear.velocity[2] = 0.0; // m/s

    task_spec.orientation.enabled = true;
    task_spec.orientation.mode = OrientationMode::Velocity;
    task_spec.orientation.segment_index = 8; // control orientation at the end-effector
    task_spec.orientation.ang_vel[0] = 0.0;  // rad/s
    task_spec.orientation.ang_vel[1] = 0.0;  // rad/s
    task_spec.orientation.ang_vel[2] = 0.0;  // rad/s
  }
  else{
    task_spec.ee_linear.enabled = true;
    task_spec.ee_linear.mode[0] = LinearMode::Position;
    task_spec.ee_linear.mode[1] = LinearMode::Position;
    task_spec.ee_linear.mode[2] = LinearMode::Position;
    task_spec.ee_linear.position[0] = arm_kinematics_->pose().p.x(); // m
    task_spec.ee_linear.position[1] = arm_kinematics_->pose().p.y(); // m
    task_spec.ee_linear.position[2] = arm_kinematics_->pose().p.z() + 0.20; // m
    task_spec.ee_linear.vel_threshold = 0.1; // m/s

    task_spec.orientation.enabled = true;
    task_spec.orientation.mode = OrientationMode::Position;
    task_spec.orientation.segment_index = 8; // control orientation at the end-effector

    double roll, pitch, yaw;
    arm_kinematics_->pose().M.GetRPY(roll, pitch, yaw);

    task_spec.orientation.rpy[0] = roll;  // roll
    task_spec.orientation.rpy[1] = pitch; // pitch
    task_spec.orientation.rpy[2] = yaw;   // yaw
  }

  // TODO: POST condition for task status
  /* If force spike observed when holding tray: human_initiation = true
     If persistent constant force equal to object weight observed when holding tray: obj_held_by_human = false
     If object reaches desired position: task_completion = true */

}

void FSMInterface::execute(events *eventData, SystemState& system_state){

  printf("\n\n");
  if (false) printf("[EE pose]: [%6.2f, %6.2f, %6.2f]\n", arm_kinematics_->pose().p.x(), arm_kinematics_->pose().p.y(), arm_kinematics_->pose().p.z());
  
  std::array<double, 6> corrected_ft_wrt_init_ref{}; // get deviation of current value from initial reference window (when ft estimator is reset)
  std::array<double, 6> corrected_ft_wrt_prev_ref{}; // get deviation of current value from previous rolling window
  const bool has_corrected_wrench = update_ft_force_estimate(system_state, 
                                                            corrected_ft_wrt_init_ref, 
                                                            corrected_ft_wrt_prev_ref);

  // selecting any of these two to test human interaction detection
  auto ft_readings_deviation = corrected_ft_wrt_prev_ref;
  // auto ft_readings_deviation = corrected_ft_wrt_init_ref;


  // check if solvers should be used based on task specification
  if (task_spec.ee_linear.enabled || task_spec.orientation.enabled || 
     task_spec.link_force.enabled || task_spec.forearm_yaw_control_enabled) {

    solver_->setState(system_state);

    if (task_spec.collaborate_spec.enabled)
    {
      if (!system_state.ft_sensor.present) {
        printf("[Warning] Collaboration enabled but FT sensor not present\n");
      }
      else if (!has_corrected_wrench)
      {
        printf("[Collaborate] Waiting until corrected wrench is available\n");
      }
      
      // ************** Monitoring of ft-sensor readings **************
      // get magnitude of corrected external force for interaction detection
      double corrected_external_force_magnitude = std::sqrt(
        ft_readings_deviation[0] * ft_readings_deviation[0] +
        ft_readings_deviation[1] * ft_readings_deviation[1] +
        ft_readings_deviation[2] * ft_readings_deviation[2]);

      human_interaction_monitoring(corrected_external_force_magnitude);

      if (human_interaction_detected){
        // if human interation is detected, switch to collaboration gains 
        // and set follow_trajectory to false and is_trajectory_computed to false
        // to re-calculate trajectory when human interation is not detected anymore
        compute_ctr_cmd_obj.setGains(ctr_config_collaborate.controllers());
        task_spec.follow_trajectory = false;
        is_trajectory_computed_ = false;

        task_spec.ee_linear.enabled = true;
        task_spec.ee_linear.mode[0] = LinearMode::Velocity;
        task_spec.ee_linear.mode[1] = LinearMode::Velocity;
        task_spec.ee_linear.mode[2] = LinearMode::Velocity;
        task_spec.ee_linear.velocity[0] = 0.0; // m/s
        task_spec.ee_linear.velocity[1] = 0.0; // m/s
        task_spec.ee_linear.velocity[2] = 0.0; // m/s
        
        task_spec.orientation.enabled = true;
        task_spec.orientation.mode = OrientationMode::Position;
        task_spec.orientation.segment_index = 8; // control orientation at the end-effector
        task_spec.orientation.rpy[0] = -M_PI / 2;
        task_spec.orientation.rpy[1] = 0.0;
        task_spec.orientation.rpy[2] = -M_PI / 2;
      }
      else if (!human_interaction_detected){
        compute_ctr_cmd_obj.setGains(ctr_config_traj_tracking.controllers());
        if (!is_trajectory_computed_) {
          compute_trajectory();
          is_trajectory_computed_ = true;
        }
        if (!task_spec.follow_trajectory) {
          task_spec.follow_trajectory = true;
        }
      }
    }

    if (task_spec.collaborate_spec.enabled && task_spec.follow_trajectory) 
    {
      if (!is_trajectory_computed_) {
        compute_trajectory();
        is_trajectory_computed_ = true;
      }

      if (trajectory_ == nullptr) {
        printf("[Error] Trajectory is null\n");
        return;
      }
      const auto now = std::chrono::steady_clock::now();
      const double elapsed_time_s = std::chrono::duration<double>(now - trajectory_start_time_).count();
      const double trajectory_duration_s = trajectory_->Duration();
      const double clamped_time_s = std::max(0.0, std::min(elapsed_time_s, trajectory_duration_s));
      // print percentage of time elapsed
      printf("\n[Trajectory] Elapsed time: %6.2f / %6.2f seconds (%3.0f%%)\n", elapsed_time_s, trajectory_duration_s, (elapsed_time_s / trajectory_duration_s) * 100.0);

      const KDL::Twist desired_twist = trajectory_->Vel(clamped_time_s);
      const KDL::Frame desired_pose = trajectory_->Pos(clamped_time_s);

      task_spec.ee_linear.enabled = true;
      task_spec.ee_linear.mode[0] = LinearMode::Velocity;
      task_spec.ee_linear.mode[1] = LinearMode::Velocity;
      task_spec.ee_linear.mode[2] = LinearMode::Velocity;
      task_spec.ee_linear.velocity[0] = desired_twist.vel.x();
      task_spec.ee_linear.velocity[1] = desired_twist.vel.y();
      task_spec.ee_linear.velocity[2] = desired_twist.vel.z();
      printf("[Trajectory] Desired EE v'ty: [%6.2f, %6.2f, %6.2f]\n", desired_twist.vel.x(), desired_twist.vel.y(), desired_twist.vel.z());
      printf("[Trajectory] Current EE v'ty: [%6.2f, %6.2f, %6.2f]\n", arm_kinematics_->twist().vel.x(), arm_kinematics_->twist().vel.y(), arm_kinematics_->twist().vel.z());
      printf("[Trajectory] Desired EE pose: [%6.2f, %6.2f, %6.2f]\n", desired_pose.p.x(), desired_pose.p.y(), desired_pose.p.z());
      printf("[Trajectory] Current EE pose: [%6.2f, %6.2f, %6.2f]\n", arm_kinematics_->pose().p.x(), arm_kinematics_->pose().p.y(), arm_kinematics_->pose().p.z());
      printf("[Trajectory] Deviation from desired pose: [%6.2f, %6.2f, %6.2f]\n", 
            desired_pose.p.x() - arm_kinematics_->pose().p.x(), 
            desired_pose.p.y() - arm_kinematics_->pose().p.y(), 
            desired_pose.p.z() - arm_kinematics_->pose().p.z());

      task_spec.orientation.enabled = true;
      task_spec.orientation.mode = OrientationMode::Position;
      task_spec.orientation.segment_index = 8;
      double roll, pitch, yaw;
      desired_pose.M.GetRPY(roll, pitch, yaw);
      task_spec.orientation.rpy[0] = roll;
      task_spec.orientation.rpy[1] = pitch;
      task_spec.orientation.rpy[2] = yaw;
    }

    if (task_spec.collaborate_spec.enabled && !task_spec.follow_trajectory) 
    {
      auto& fext_wrenches = solver_->externalWrenches_fext_solver(); // f_ext_fext_;
      if (!fext_wrenches.empty()) 
      {
        auto& ee_external_wrench = fext_wrenches.back();
        for (int axis = 0; axis < 6; ++axis) ee_external_wrench(axis) = 0.0;

        if (true) 
        {
          printf("[Collaborate] Reference wrench: fx=%6.2f, fy=%6.2f, fz=%6.2f, tx=%6.2f, ty=%6.2f, tz=%6.2f\n",
                ft_reference_mean_[0], ft_reference_mean_[1], ft_reference_mean_[2],
                ft_reference_mean_[3], ft_reference_mean_[4], ft_reference_mean_[5]);
          printf("[Collaborate] Corrected wrench: fx=%6.2f, fy=%6.2f, fz=%6.2f, tx=%6.2f, ty=%6.2f, tz=%6.2f\n",
                ft_readings_deviation[0], ft_readings_deviation[1], ft_readings_deviation[2],
                ft_readings_deviation[3], ft_readings_deviation[4], ft_readings_deviation[5]);
        }

        if (has_corrected_wrench) 
        {
          const double scale            = task_spec.collaborate_spec.magnification_factor;
          const double saturation_limit = task_spec.collaborate_spec.f_ext_saturation_limit;

          for (int axis = 0; axis < 3; ++axis) { // TODO? only applying external wrench in linear axes for now
            ft_readings_deviation[axis] = std::max(0.0, std::abs(ft_readings_deviation[axis]) - task_spec.collaborate_spec.external_force_deadband) *
                                            ((ft_readings_deviation[axis] > 0) ? 1.0 : -1.0);
            const double external_wrench_unsat = scale * ft_readings_deviation[axis];
            ee_external_wrench(axis) = std::max(std::min(external_wrench_unsat, saturation_limit), -saturation_limit);
          }
        }
      }

      solver_->computeTorquesFext();
    }

    // get beta and f_ext based on task_spec and current state using controllers
    DebugSample debug_sample;
    compute_ctr_cmd_obj.compute(
        system_state,
        *arm_kinematics_,
        task_spec,
        solver_->beta(),
        solver_->externalWrenches(),
        system_state.arm.cycle_time,
        &debug_sample);

    for (int i = 0; i < NUM_JOINTS; ++i) {
      debug_sample.joint_position[i] = system_state.arm.q[i];
      debug_sample.joint_velocity[i] = system_state.arm.qd[i];
    }
    debug_sample.sample_time = std::chrono::steady_clock::now();
    debug_sample.sequence = ++debug_sequence_counter_;
    debug_sample.fsm_state = static_cast<int>(fsm_execution_state);
    latest_debug_sample_ = debug_sample;
    debug_sample_valid_ = true;

    printf("[EE Velocity] : [%6.2f, %6.2f, %6.2f]\n", arm_kinematics_->twist().vel.x(), arm_kinematics_->twist().vel.y(), arm_kinematics_->twist().vel.z());
    printf("[EE pose] : [%6.2f, %6.2f, %6.2f]\n", arm_kinematics_->pose().p.x(), arm_kinematics_->pose().p.y(), arm_kinematics_->pose().p.z());
    printf("[EE orientation RPY] : [%6.2f, %6.2f, %6.2f]\n", arm_kinematics_->rpy()[0], arm_kinematics_->rpy()[1], arm_kinematics_->rpy()[2]);
    printf("[Beta command] : [%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f]\n", solver_->beta()(0), solver_->beta()(1), solver_->beta()(2), solver_->beta()(3), solver_->beta()(4), solver_->beta()(5));
    printf("[External wrench command @ EE] : fx=%6.2f, fy=%6.2f, fz=%6.2f, tx=%6.2f, ty=%6.2f, tz=%6.2f\n",
            solver_->externalWrenches().back()(0),
            solver_->externalWrenches().back()(1),
            solver_->externalWrenches().back()(2),
            solver_->externalWrenches().back()(3),
            solver_->externalWrenches().back()(4),
            solver_->externalWrenches().back()(5));

    // computeTorques adds torques from vn_fixed_joint and vn_fext solvers

    /*
    // Using V'n solver
    set linear external wrenches at the end-effector to zero, as it is currently aded to be used for rnea
    for (int axis = 0; axis < 3; ++axis) {
      solver_->externalWrenches().back()(axis) = 0.0;
    }
    solver_->computeTorques();
    solver_->updateTorqueCmdInState(system_state);
    */

    // /*
    // Using RNEA solver
    auto& fext_wrenches_rnea = solver_->externalWrenches_rnea(); // f_ext_rnea_;
    // to test, set external wrench for RNEA solver to be the same as V'n fixed jnt solver
    for (size_t i = 0; i < fext_wrenches_rnea.size(); ++i) {
      fext_wrenches_rnea[i] = solver_->externalWrenches()[i];
    }

    // for end-effector, add external wrench calculated from collaborate behavior if enabled
    if (task_spec.collaborate_spec.enabled && has_corrected_wrench)
    {
      auto& ee_external_wrench_rnea = fext_wrenches_rnea.back();
      for (int axis = 0; axis < 6; ++axis) {
        // negative sign: RNEA assumes resisting external wrenches;
        // V'n uses the opposite convention
        ee_external_wrench_rnea(axis) -= solver_->externalWrenches_fext_solver().back()(axis);
      }
    }

    // transform wrenches from base link to corresponding segements for forearm and end-effector
    fext_wrenches_rnea[2] = arm_kinematics_->forearmPoseBL().M.Inverse() * fext_wrenches_rnea[2]; // transform to forearm frame
    fext_wrenches_rnea.back() = arm_kinematics_->pose().M.Inverse() * fext_wrenches_rnea.back();

    solver_->computeTorquesRNEA(arm_kinematics_->jointVelocities(),
                                arm_kinematics_->jointPositions(),
                                arm_kinematics_->jointVelocity(),
                                solver_->externalWrenches_rnea());
    
    solver_->updateTorqueCmdFromRNEAInState(system_state);

    // */

    // reset torque commands to zero from all solvers
    solver_->resetTorqueOutputs();
  }
  else if (task_spec.joint_position.enabled) {
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      double position_error = task_spec.joint_position.position[i] - system_state.arm.q[i];
      normalize_angle_diff(position_error);
      if (false) printf("Joint %d position error: %6.2f\n", i, position_error);
      system_state.arm.tau_cmd[i] = 70.0 * position_error;
    }
  }
  else {
    printf("No valid command specified in task spec\n");
    produce_event(eventData, E_ENTER_EXIT);
    return;
  }

  // clamp joint torques
  for (int i = 0; i < NUM_JOINTS; ++i)
  {
    if (false) {
      printf("[Joint torque] capped at %6.2f; [%d]: %6.2f\n", KINOVA_TAU_CMD_LIMIT, i, system_state.arm.tau_cmd[i]);
    }
    avoid_joint_limits(system_state);
    system_state.arm.tau_cmd[i] = std::max(std::min(system_state.arm.tau_cmd[i], KINOVA_TAU_CMD_LIMIT), -KINOVA_TAU_CMD_LIMIT);
    // set torque to zero while testing
    // system_state.arm.tau_cmd[i] = 0.0;
  }

  // send control commands to arm and update arm_kinematics state
  robif2b_kinova_gen3_update(&rob);
  arm_kinematics_->update(system_state);
  if (system_state.ft_sensor.present) {
    transform_ft_readings_to_BL_update_state(system_state, *arm_kinematics_);
  }

  // send gripper command if enabled in task spec and gripper is present
  if (task_spec.gripper.enabled)
  {
    if (system_state.gripper.present)
    {
      system_state.gripper.pos_cmd[0] = task_spec.gripper.position;
      robif2b_kg3_robotiq_gripper_update(&gripper);
    }
    else
    {
      printf("Gripper command specified but gripper not present\n");
    }
  }

  // FT sensor is updated in a separate 100 Hz thread. Using cached state
  if (false && system_state.ft_sensor.present)
  {
    printf("FT sensor readings: fx=%6.2f, fy=%6.2f, fz=%6.2f, tx=%6.2f, ty=%6.2f, tz=%6.2f\n", 
      system_state.ft_sensor.fx, system_state.ft_sensor.fy, system_state.ft_sensor.fz, 
      system_state.ft_sensor.tx, system_state.ft_sensor.ty, system_state.ft_sensor.tz);
  }

  check_post_condition(eventData, system_state, task_spec);
}

void FSMInterface::touch_table_behavior_config(events *eventData, SystemState& system_state){
  
  fsm_execution_state = S_M_TOUCH_TABLE;
  compute_ctr_cmd_obj.setGains(ctr_config_touch_table.controllers());
  task_spec.resetDefault();


  task_spec.forearm_yaw_control_enabled = true;
  task_spec.ee_linear.enabled = true;
  task_spec.ee_linear.mode[0] = LinearMode::Velocity;
  task_spec.ee_linear.mode[1] = LinearMode::Velocity;
  task_spec.ee_linear.mode[2] = LinearMode::Velocity;

  task_spec.ee_linear.velocity[0] = 0.0; // m/s
  task_spec.ee_linear.velocity[1] = 0.0; // m/s
  task_spec.ee_linear.velocity[2] = -0.06; // m/s

  task_spec.orientation.enabled = true;
  task_spec.orientation.mode = OrientationMode::Position;
  task_spec.orientation.segment_index = 8; // control orientation at the end-effector
  task_spec.orientation.rpy[0] = -M_PI / 2;
  task_spec.orientation.rpy[1] = 0.0;
  task_spec.orientation.rpy[2] = -M_PI / 2;

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 2;
  task_spec.post_condition.logic = LogicOp::And;

  // task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  // task_spec.post_condition.constraints[0].axis = 2;     // z-axis
  // task_spec.post_condition.constraints[0].op = CompareOp::GreaterEqual;
  // task_spec.post_condition.constraints[0].value = 0.5; // m

  task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis = 2;     // z-axis
  task_spec.post_condition.constraints[0].op = CompareOp::LessEqual;
  task_spec.post_condition.constraints[0].value = 0.06; // m

  task_spec.post_condition.constraints[1].type = ConstraintType::Velocity;
  task_spec.post_condition.constraints[1].axis = 2;     // z-axis
  task_spec.post_condition.constraints[1].op = CompareOp::LessEqual;
  task_spec.post_condition.constraints[1].value = 0.01; // m/s

  produce_event(eventData, E_M_TOUCH_TABLE_CONFIGURED);
}

void FSMInterface::slide_on_table_behavior_config(events *eventData, SystemState& system_state){

  fsm_execution_state = S_M_SLIDE_ALONG_TABLE;
  compute_ctr_cmd_obj.setGains(ctr_config_slide_on_table.controllers());
  task_spec.resetDefault();

  task_spec.forearm_yaw_control_enabled = true;
  task_spec.ee_linear.enabled = true;
  task_spec.ee_linear.mode[0] = LinearMode::Velocity;
  task_spec.ee_linear.mode[1] = LinearMode::Velocity;
  task_spec.ee_linear.mode[2] = LinearMode::Force;
  task_spec.ee_linear.velocity[0] = 0.02;     // m/s
  task_spec.ee_linear.velocity[1] = 0.0;      // m/s
  task_spec.ee_linear.force[2]    = -5.0;     // N

  task_spec.orientation.enabled = true;
  task_spec.orientation.mode = OrientationMode::Position;
  task_spec.orientation.segment_index = 8; // control orientation at the end-effector
  task_spec.orientation.rpy[0] = -M_PI / 2;
  task_spec.orientation.rpy[1] = 0.0;
  task_spec.orientation.rpy[2] = -M_PI / 2;

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 1;
  task_spec.post_condition.logic = LogicOp::And;

  task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis = 0; // x-axis
  task_spec.post_condition.constraints[0].op = CompareOp::GreaterEqual;
  task_spec.post_condition.constraints[0].value = 0.6; // m

  // task_spec.post_condition.constraints[1].type = ConstraintType::Position;
  // task_spec.post_condition.constraints[1].axis = 2; // z-axis
  // task_spec.post_condition.constraints[1].op = CompareOp::LessEqual;
  // task_spec.post_condition.constraints[1].value = 0.06; // m

  produce_event(eventData, E_M_SLIDE_ALONG_TABLE_CONFIGURED);
}

void FSMInterface::grasp_object_behavior_config(events *eventData, SystemState& system_state){

  fsm_execution_state = S_M_GRASP_OBJECT;
  compute_ctr_cmd_obj.setGains(ctr_config_grasp_object.controllers());
  task_spec.resetDefault();

  task_spec.forearm_yaw_control_enabled = true;
  task_spec.ee_linear.enabled = true;
  task_spec.ee_linear.mode[0] = LinearMode::Velocity;
  task_spec.ee_linear.mode[1] = LinearMode::Velocity;
  task_spec.ee_linear.mode[2] = LinearMode::Velocity;

  task_spec.ee_linear.velocity[0] = 0.0; // m/s
  task_spec.ee_linear.velocity[1] = 0.0; // m/s
  task_spec.ee_linear.velocity[2] = 0.0; // m/s

  task_spec.orientation.enabled = true;
  task_spec.orientation.mode = OrientationMode::Position;
  task_spec.orientation.segment_index = 8; // control orientation at the end-effector
  task_spec.orientation.rpy[0] = -M_PI / 2;
  task_spec.orientation.rpy[1] = 0.0;
  task_spec.orientation.rpy[2] = -M_PI / 2;

  task_spec.gripper.enabled = true;
  task_spec.gripper.position = 95.0; // fully closed

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 1;

  task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis = 3; // gripper axis
  task_spec.post_condition.constraints[0].op = CompareOp::GreaterEqual;
  task_spec.post_condition.constraints[0].value = 95.0; // fully closed

  produce_event(eventData, E_M_GRASP_OBJECT_CONFIGURED);
}

void FSMInterface::collaborate_behavior_config(events *eventData, SystemState& system_state){

  fsm_execution_state = S_M_COLLABORATE;
  compute_ctr_cmd_obj.setGains(ctr_config_traj_tracking.controllers());
  // compute_ctr_cmd_obj.setGains(ctr_config_collaborate.controllers());
  task_spec.resetDefault();

  // Compute trajectory on state entry using current EE pose as the start pose.
  if (!arm_kinematics_) {
    trajectory_object_.reset();
    trajectory_ = nullptr;
    is_trajectory_computed_ = false;
    printf("[Collaborate] Cannot compute trajectory: arm kinematics is not initialized\n");
  } else if (!is_trajectory_computed_) {
    compute_trajectory();
    is_trajectory_computed_ = true;
  }

  // print trajectory metadata
  if (trajectory_) {
    printf("[Collaborate] Trajectory computed with duration %6.2f seconds\n", trajectory_->Duration());
  } else {
    printf("[Collaborate] No trajectory available\n");
  }

  task_spec.forearm_yaw_control_enabled = true;
  task_spec.ee_linear.enabled = true;
  task_spec.ee_linear.mode[0] = LinearMode::Velocity;
  task_spec.ee_linear.mode[1] = LinearMode::Velocity;
  task_spec.ee_linear.mode[2] = LinearMode::Velocity;

  task_spec.ee_linear.velocity[0] = 0.0; // m/s
  task_spec.ee_linear.velocity[1] = 0.0; // m/s
  task_spec.ee_linear.velocity[2] = 0.0; // m/s

  task_spec.orientation.enabled = true;
  task_spec.orientation.mode = OrientationMode::Position;
  task_spec.orientation.segment_index = 8; // control orientation at the end-effector
  task_spec.orientation.rpy[0] = -M_PI / 2;
  task_spec.orientation.rpy[1] = 0.0;
  task_spec.orientation.rpy[2] = -M_PI / 2;

  task_spec.follow_trajectory = true;
  task_spec.collaborate_spec.enabled = true; // current logic: if enabled, start traj following, then on human intervention, switch to collaboration
  task_spec.collaborate_spec.magnification_factor = 5.0;     // scale
  task_spec.collaborate_spec.external_force_deadband  = 3.0; // N
  task_spec.collaborate_spec.f_ext_saturation_limit = 10.0;  // N

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 1;

  task_spec.post_condition.constraints[0].type  = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis  = 2; // z-axis
  task_spec.post_condition.constraints[0].op    = CompareOp::LessEqual;
  task_spec.post_condition.constraints[0].value = 0.01; // m

  produce_event(eventData, E_M_COLLABORATE_CONFIGURED);
}

void FSMInterface::release_object_behavior_config(events *eventData, SystemState& system_state){
  
  fsm_execution_state = S_M_RELEASE_OBJECT;
  compute_ctr_cmd_obj.setGains(ctr_config_release_object.controllers());
  task_spec.resetDefault();

  task_spec.forearm_yaw_control_enabled = true;
  task_spec.ee_linear.enabled = true;
  task_spec.ee_linear.mode[0] = LinearMode::Velocity;
  task_spec.ee_linear.mode[1] = LinearMode::Velocity;
  task_spec.ee_linear.mode[2] = LinearMode::Velocity;

  task_spec.ee_linear.velocity[0] = 0.0; // m/s
  task_spec.ee_linear.velocity[1] = 0.0; // m/s
  task_spec.ee_linear.velocity[2] = 0.0; // m/s

  task_spec.orientation.enabled = true;
  task_spec.orientation.mode = OrientationMode::Position;
  task_spec.orientation.segment_index = 8; // control orientation at the end-effector
  task_spec.orientation.rpy[0] = -M_PI / 2;
  task_spec.orientation.rpy[1] = 0.0;
  task_spec.orientation.rpy[2] = -M_PI / 2;

  task_spec.gripper.enabled = true;
  task_spec.gripper.position = 0.0; // fully open

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 1;

  task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis = 3;     // gripper axis
  task_spec.post_condition.constraints[0].op = CompareOp::LessEqual;
  task_spec.post_condition.constraints[0].value = 95.0;  // fully closed

  produce_event(eventData, E_M_RELEASE_OBJECT_CONFIGURED);
}

void FSMInterface::exit(events *eventData, SystemState& system_state){

  fsm_execution_state = S_EXIT;
  // stop the arm and shutdown communication
  if (in_comm_with_hw == true) {
    if (system_state.ft_sensor.present) {
      std::cout << "Stopping ft_sensor..." << std::endl;
      robif2b_robotiq_ft_stop(&ft_sensor);
      robif2b_robotiq_ft_shutdown(&ft_sensor);
    }
    if (system_state.gripper.present) {
      std::cout << "Stopping gripper..." << std::endl;
      robif2b_kg3_robotiq_gripper_stop(&gripper);
    }
    std::cout << "Shutting down arm..." << std::endl;
    robif2b_kinova_gen3_stop(&rob);
    robif2b_kinova_gen3_shutdown(&rob);
    if (!rob.success) {
      printf("Error while shutting down gen3\n");
    }
  // check: 
  }
  in_comm_with_hw = false;
}

// decision of which behavior to execute based on events
void FSMInterface::fsm_behavior(events *eventData, SystemState& system_state){

  if (consume_event(eventData, E_ENTER_IDLE)) {
    idle(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_CONFIGURE)) {
    configure(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_EXECUTE)) {
    execute(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_TOUCH_TABLE)) {
    touch_table_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_SLIDE_ALONG_TABLE)) {
    slide_on_table_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_GRASP_OBJECT)) {
    grasp_object_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_COLLABORATE)) {
    collaborate_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_M_RELEASE_OBJECT)) {
    release_object_behavior_config(eventData, system_state);
  }
  if (consume_event(eventData, E_ENTER_EXIT)) {
    exit(eventData, system_state);
  }
}

void FSMInterface::run_fsm(){
  produce_event(&eventData, E_STEP);
  fsm_behavior(&eventData, system_state);
  fsm_step_nbx(&fsm);
  reconfig_event_buffers(&eventData);
};

bool FSMInterface::getLatestDebugSample(DebugSample& out) const
{
  if (!debug_sample_valid_) {
    return false;
  }
  out = latest_debug_sample_;
  return true;
}