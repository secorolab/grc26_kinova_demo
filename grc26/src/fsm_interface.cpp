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
  if (!task_triggered) 
  {
    task_triggered = true;
    // produce_event(eventData, E_M_TOUCH_TABLE_CONFIG);
    produce_event(eventData, E_M_COLLABORATE_CONFIG);
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

  std::array<double, 6> corrected_external_wrench{};
  if (false) printf("ft_sensor readings: fx=%f, fy=%f, fz=%f, tx=%f, ty=%f, tz=%f\n",
            system_state.ft_sensor.wrench[0],
            system_state.ft_sensor.wrench[1],
            system_state.ft_sensor.wrench[2],
            system_state.ft_sensor.wrench[3],
            system_state.ft_sensor.wrench[4],
            system_state.ft_sensor.wrench[5]);
  const bool has_corrected_wrench = update_ft_force_estimate(system_state, corrected_external_wrench);

  if (false) printf("EE pose: [%f, %f, %f]\n", arm_kinematics_->pose().p.x(), arm_kinematics_->pose().p.y(), arm_kinematics_->pose().p.z());

  if (task_spec.ee_linear.enabled || task_spec.orientation.enabled || 
     task_spec.link_force.enabled || task_spec.forearm_yaw_control_enabled) {

    solver_->setState(system_state);

    // print joint positions
    if (false) {
      printf("Joint positions: [");
      for (int i = 0; i < NUM_JOINTS; ++i) {
        printf("%f", system_state.arm.q[i]);
        if (i < NUM_JOINTS - 1) {
          printf(", ");
        }
      }
      printf("]\n");
    }

    if (task_spec.collaborate_spec.enabled) {
      if (!system_state.ft_sensor.present) {
        printf("[Warning] Collaboration enabled but FT sensor not present\n");
      }
    }
    if (task_spec.collaborate_spec.enabled && has_corrected_wrench) 
    {
      auto& fext_wrenches =  solver_->externalWrenches_fext_solver(); // f_ext_fext_;
      if (!fext_wrenches.empty()) 
      {
        auto& ee_external_wrench = fext_wrenches.back();
        for (int axis = 0; axis < 6; ++axis) ee_external_wrench(axis) = 0.0;

        if (true) 
        {
          printf("Reference external wrench: fx=%8.2f, fy=%8.2f, fz=%8.2f, tx=%8.2f, ty=%8.2f, tz=%8.2f\n",
                ft_reference_mean_[0], ft_reference_mean_[1], ft_reference_mean_[2],
                ft_reference_mean_[3], ft_reference_mean_[4], ft_reference_mean_[5]);
          printf("Corrected external wrench: fx=%8.2f, fy=%8.2f, fz=%8.2f, tx=%8.2f, ty=%8.2f, tz=%8.2f\n",
                corrected_external_wrench[0], corrected_external_wrench[1], corrected_external_wrench[2],
                corrected_external_wrench[3], corrected_external_wrench[4], corrected_external_wrench[5]);
        }

        if (has_corrected_wrench) 
        {
          const double scale            = task_spec.collaborate_spec.magnification_factor;
          const double saturation_limit = task_spec.collaborate_spec.f_ext_saturation_limit;

          for (int axis = 0; axis < 3; ++axis) { // TODO? only apply external wrench in linear axes for now
            corrected_external_wrench[axis] = std::max(0.0, std::abs(corrected_external_wrench[axis]) - task_spec.collaborate_spec.external_force_deadband) *
                                             ((corrected_external_wrench[axis] > 0) ? 1.0 : -1.0);
            const double external_wrench_unsat = scale * corrected_external_wrench[axis];
            ee_external_wrench(axis) = std::max(std::min(external_wrench_unsat, saturation_limit), -saturation_limit);
            printf("Axis %d: raw=%8.2f, corrected=%8.2f, unsat_cmd=%8.2f, final_cmd=%8.2f\n", axis, system_state.ft_sensor.wrench_BL[axis], corrected_external_wrench[axis], external_wrench_unsat, ee_external_wrench(axis));
            if (true) {
              printf("[1] Applied external wrench on axis %d: %f\n", axis, ee_external_wrench(axis));
            }
          }
        }
      }

      solver_->computeTorquesFext();
      if (true) {
        for (int i = 0; i < 7; ++i) {
          printf("[Collaborate] Joint torque from fext solver on joint %d: %f\n", i, solver_->tauCmdFext()(i));
        }
      }
    }
    // get beta and f_ext based on task_spec and current state using controllers
    compute_ctr_cmd_obj.compute(
        system_state,
        *arm_kinematics_,
        task_spec,
        solver_->beta(),
        solver_->externalWrenches());

    printf("[EE Velocity] : [%f, %f, %f]\n", arm_kinematics_->twist().vel.x(), arm_kinematics_->twist().vel.y(), arm_kinematics_->twist().vel.z());
    printf("[Beta command] : [%f, %f, %f, %f, %f, %f]\n", solver_->beta()(0), solver_->beta()(1), solver_->beta()(2), solver_->beta()(3), solver_->beta()(4), solver_->beta()(5));
    printf("[External wrench command @ EE] : fx=%f, fy=%f, fz=%f, tx=%f, ty=%f, tz=%f\n",
            solver_->externalWrenches().back()(0),
            solver_->externalWrenches().back()(1),
            solver_->externalWrenches().back()(2),
            solver_->externalWrenches().back()(3),
            solver_->externalWrenches().back()(4),
            solver_->externalWrenches().back()(5));

    // computeTorques adds torques from vn_fixed_joint and vn_fext solvers
    solver_->computeTorques();
    solver_->updateTorqueCmdInState(system_state);
    solver_->resetTorqueOutputs();
  }
  else if (task_spec.joint_position.enabled) {
    for (int i = 0; i < NUM_JOINTS; ++i)
    {
      double position_error = task_spec.joint_position.position[i] - system_state.arm.q[i];
      normalize_angle_diff(position_error);
      if (false) printf("Joint %d position error: %f\n", i, position_error);
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
      printf("[Joint torque] capped at %f; [%d]: %f\n", KINOVA_TAU_CMD_LIMIT, i, system_state.arm.tau_cmd[i]);
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
    printf("FT sensor readings: fx=%f, fy=%f, fz=%f, tx=%f, ty=%f, tz=%f\n", 
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
  task_spec.ee_linear.velocity[2] = 0.02; // m/s

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
  task_spec.post_condition.constraints[0].axis = 2;     // z-axis
  task_spec.post_condition.constraints[0].op = CompareOp::GreaterEqual;
  task_spec.post_condition.constraints[0].value = 0.5; // m

  // task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  // task_spec.post_condition.constraints[0].axis = 2;     // z-axis
  // task_spec.post_condition.constraints[0].op = CompareOp::LessEqual;
  // task_spec.post_condition.constraints[0].value = 0.06; // m

  // task_spec.post_condition.constraints[1].type = ConstraintType::Velocity;
  // task_spec.post_condition.constraints[1].axis = 2;     // z-axis
  // task_spec.post_condition.constraints[1].op = CompareOp::LessEqual;
  // task_spec.post_condition.constraints[1].value = 0.01; // m/s

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
  compute_ctr_cmd_obj.setGains(ctr_config_collaborate.controllers());
  task_spec.resetDefault();

  // TODO: define collaborate behavior spec
  task_spec.forearm_yaw_control_enabled = true;
  task_spec.ee_linear.enabled = true;
  task_spec.ee_linear.mode[0] = LinearMode::Velocity;
  task_spec.ee_linear.mode[1] = LinearMode::Velocity;
  task_spec.ee_linear.mode[2] = LinearMode::Velocity;

  task_spec.ee_linear.velocity[0] = 0.0; // m/s
  task_spec.ee_linear.velocity[1] = 0.0; // m/s
  task_spec.ee_linear.velocity[2] = -0.02; // m/s

  task_spec.orientation.enabled = true;
  task_spec.orientation.mode = OrientationMode::Position;
  task_spec.orientation.segment_index = 8; // control orientation at the end-effector
  task_spec.orientation.rpy[0] = -M_PI / 2;
  task_spec.orientation.rpy[1] = 0.0;
  task_spec.orientation.rpy[2] = -M_PI / 2;

  task_spec.collaborate_spec.enabled = false;
  task_spec.collaborate_spec.magnification_factor = 7.0;     // scale
  task_spec.collaborate_spec.external_force_deadband  = 7.0; // N
  task_spec.collaborate_spec.f_ext_saturation_limit = 15.0;  // N

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 1;

  task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis = 2; // z-axis
  task_spec.post_condition.constraints[0].op = CompareOp::LessEqual;
  task_spec.post_condition.constraints[0].value = 0.03; // m

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