#include "grc26/fsm_interface.hpp"

#define KINOVA_TAU_CMD_LIMIT 30.0

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
    compute_ctr_cmd_obj(config.controllers()),
    in_comm_with_hw(false)
{}

FSMInterface::~FSMInterface() {
}

int FSMInterface::get_current_state() const
{
    return fsm.currentStateIndex;
}

bool FSMInterface::is_in_comm_with_hw() const
{
    return in_comm_with_hw;
}

void FSMInterface::configure(events *eventData, SystemState& system_state){

  // initialise KDL model of the arm from URDF
  model_ = std::make_unique<ArmKDLModel>();
  bool kdl_model_loaded =
    model_->loadFromURDF("grc26",
                    "GEN3_URDF_V12.urdf",
                    "base_link",
                    "EndEffector_Link");

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
  config.load("controller_gains.yaml");
  compute_ctr_cmd_obj.setGains(config.controllers());

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
        printf("Stopped ft sensor\n");
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
  // emit event to transition to idle state after configuration
  produce_event(eventData, E_CONFIGURED);
}

void FSMInterface::idle(events *eventData, const SystemState& system_state){

  task_status.idle = true;
  robif2b_kinova_gen3_update(&rob);
  if (system_state.ft_sensor.present)
  {
    robif2b_robotiq_ft_update(&ft_sensor);
  }

  arm_kinematics_->update(system_state);
  // Optionally, update alpha for solver in all 6 axes
  /* KDL::Jacobian alpha_ = KDL::Jacobian(6);
     alpha_.setColumn(0, KDL::Twist(KDL::Vector(1, 0, 0), KDL::Vector(0, 0, 0)));
     solver_->setAlpha(alpha_);
  */
  compute_ctr_cmd_obj.setGains(config.controllers());
  task_spec.resetDefault();

  produce_event(eventData, E_M_TOUCH_TABLE_CONFIG);

  task_spec.ee_linear.enabled = true;
  task_spec.ee_linear.mode[0] = LinearMode::Position;
  task_spec.ee_linear.mode[1] = LinearMode::Position;
  task_spec.ee_linear.mode[2] = LinearMode::Position;
  task_spec.ee_linear.position[0] = arm_kinematics_->pose().p.x(); // m
  task_spec.ee_linear.position[1] = arm_kinematics_->pose().p.y(); // m
  task_spec.ee_linear.position[2] = arm_kinematics_->pose().p.z(); // m

  task_spec.orientation.enabled = true;
  task_spec.orientation.mode = OrientationMode::Position;
  task_spec.orientation.segment_index = 7; // control orientation at the end-effector

  double roll, pitch, yaw;
  arm_kinematics_->pose().M.GetRPY(roll, pitch, yaw);

  task_spec.orientation.rpy[0] = roll; // roll
  task_spec.orientation.rpy[1] = pitch; // pitch
  task_spec.orientation.rpy[2] = yaw; // yaw

  // TODO: POST condition for task status
  // If force spike observed when holding tray: human_initiation = true
  // If persistent constant force equal to object weight observed when holding tray: obj_held_by_human = fals
  // If object reaches desired position: task_completion = true


}

void FSMInterface::execute(events *eventData, SystemState& system_state){

  // solve for control commands
  compute_ctr_cmd_obj.compute(
      system_state, 
      *arm_kinematics_, 
      task_spec, 
      solver_->beta(), 
      solver_->externalWrenches());

  solver_->setState(system_state);
  solver_->computeTorques();
  solver_->updateTorqueCmdInState(system_state);

  // send control commands to robot
  robif2b_kinova_gen3_update(&rob);
  arm_kinematics_->update(system_state);

  if (task_spec.gripper.enabled)
  {
    if (task_spec.gripper.enabled)
    {
      system_state.gripper.pos_cmd[0] = task_spec.gripper.position;
    }
    robif2b_kg3_robotiq_gripper_update(&gripper);
  }

  if (system_state.ft_sensor.present)
  {
    robif2b_robotiq_ft_update(&ft_sensor);
  }

  check_post_condition(eventData, system_state, task_spec);
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
        if (constraint.axis < 3) // linear axis
          value = arm_kinematics_->pose().p[constraint.axis];
        else if (constraint.axis == 3) // gripper
          value = system_state.gripper.pos_msr[0];
        break;

      case ConstraintType::Velocity:
        if (constraint.axis < 3) // linear axis
          value = arm_kinematics_->twist().vel[constraint.axis];
        else if (constraint.axis == 3) // gripper
          value = system_state.gripper.vel_msr[0];
        break;

      case ConstraintType::Force:
        if (constraint.axis < 3) // linear axis
          value = system_state.ft_sensor.wrench[constraint.axis];
        break;

      case ConstraintType::Torque:
        if (constraint.axis < 3) // linear axis
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
    if (fsm.currentStateIndex == S_M_TOUCH_TABLE)
      produce_event(eventData, E_ENTER_EXIT);
      // produce_event(eventData, E_M_SLIDE_ALONG_TABLE_CONFIG);
    else if (fsm.currentStateIndex == S_M_SLIDE_ALONG_TABLE)
      produce_event(eventData, E_M_GRASP_OBJECT_CONFIG);
    else if (fsm.currentStateIndex == S_M_GRASP_OBJECT)
      produce_event(eventData, E_ENTER_IDLE);
    else if (fsm.currentStateIndex == S_M_COLLABORATE)
      produce_event(eventData, E_M_RELEASE_OBJECT_CONFIG);
    else if (fsm.currentStateIndex == S_M_RELEASE_OBJECT)
      produce_event(eventData, E_ENTER_EXIT);
  }
}

void FSMInterface::touch_table_behavior_config(events *eventData, SystemState& system_state){
  
  compute_ctr_cmd_obj.setGains(config.controllers());
  task_spec.resetDefault();
  task_spec.ee_linear.enabled = true;
  task_spec.ee_linear.mode[0] = LinearMode::Velocity;
  task_spec.ee_linear.mode[1] = LinearMode::Velocity;
  task_spec.ee_linear.mode[2] = LinearMode::Velocity;
  task_spec.ee_linear.velocity[0] = 0.0;   // m/s
  task_spec.ee_linear.velocity[1] = 0.0;   // m/s
  task_spec.ee_linear.velocity[2] = -0.01; // m/s

  task_spec.orientation.enabled = true;
  task_spec.orientation.segment_index = 7; // control orientation at the end-effector
  task_spec.orientation.mode = OrientationMode::Position;
  task_spec.orientation.rpy[0] = 0.0; // roll
  task_spec.orientation.rpy[1] = 0.0; // pitch
  task_spec.orientation.rpy[2] = 0.0; // yaw

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 2;
  task_spec.post_condition.logic = LogicOp::And;

  task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis = 2; // z-axis
  task_spec.post_condition.constraints[0].op = CompareOp::LessEqual;
  task_spec.post_condition.constraints[0].value = 0.05; // m

  task_spec.post_condition.constraints[1].type = ConstraintType::Velocity;
  task_spec.post_condition.constraints[1].axis = 2; // z-axis
  task_spec.post_condition.constraints[1].op = CompareOp::LessEqual;
  task_spec.post_condition.constraints[1].value = 0.01; // m/s

  produce_event(eventData, E_M_TOUCH_TABLE_CONFIGURED);
}

void FSMInterface::slide_on_table_behavior_config(events *eventData, SystemState& system_state){

  compute_ctr_cmd_obj.setGains(config.controllers());
  task_spec.resetDefault();
  task_spec.ee_linear.enabled = true;
  task_spec.ee_linear.mode[0] = LinearMode::Velocity;
  task_spec.ee_linear.mode[1] = LinearMode::Velocity;
  task_spec.ee_linear.mode[2] = LinearMode::Force;
  task_spec.ee_linear.velocity[0] = 0.0;    // m/s
  task_spec.ee_linear.velocity[1] = 0.01;   // m/s
  task_spec.ee_linear.force[2] = 5.0;       // N

  task_spec.orientation.enabled = true;
  task_spec.orientation.segment_index = 7; // control orientation at the end-effector
  task_spec.orientation.mode = OrientationMode::Position;
  task_spec.orientation.rpy[0] = 0.0; // roll
  task_spec.orientation.rpy[1] = 0.0; // pitch
  task_spec.orientation.rpy[2] = 0.0; // yaw

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 2;
  task_spec.post_condition.logic = LogicOp::And;

  task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis = 2; // z-axis
  task_spec.post_condition.constraints[0].op = CompareOp::LessEqual;
  task_spec.post_condition.constraints[0].value = 0.02; // m

  task_spec.post_condition.constraints[1].type = ConstraintType::Velocity;
  task_spec.post_condition.constraints[1].axis = 2; // z-axis
  task_spec.post_condition.constraints[1].op = CompareOp::LessEqual;
  task_spec.post_condition.constraints[1].value = 0.01; // m/s

  produce_event(eventData, E_M_SLIDE_ALONG_TABLE_CONFIGURED);
}

void FSMInterface::grasp_object_behavior_config(events *eventData, SystemState& system_state){

  compute_ctr_cmd_obj.setGains(config.controllers());
  task_spec.resetDefault();
  task_spec.gripper.enabled = true;
  task_spec.gripper.position = 1.0; // fully closed

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 1;

  task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis = 3; // gripper axis
  task_spec.post_condition.constraints[0].op = CompareOp::GreaterEqual;
  task_spec.post_condition.constraints[0].value = 0.95; // fully closed

  produce_event(eventData, E_M_GRASP_OBJECT_CONFIGURED);
}

void FSMInterface::collaborate_behavior_config(events *eventData, SystemState& system_state){

  compute_ctr_cmd_obj.setGains(config.controllers());
  task_spec.resetDefault();
  // TODO
  
  produce_event(eventData, E_M_COLLABORATE_CONFIGURED);
}

void FSMInterface::release_object_behavior_config(events *eventData, SystemState& system_state){
  
  compute_ctr_cmd_obj.setGains(config.controllers());
  task_spec.resetDefault();
  task_spec.gripper.enabled = true;
  task_spec.gripper.position = 0.0; // fully open

  task_spec.post_condition.available = true;
  task_spec.post_condition.num_constraints = 1;

  task_spec.post_condition.constraints[0].type = ConstraintType::Position;
  task_spec.post_condition.constraints[0].axis = 3; // gripper axis
  task_spec.post_condition.constraints[0].op = CompareOp::LessEqual;
  task_spec.post_condition.constraints[0].value = 0.1; // fully closed

  produce_event(eventData, E_M_RELEASE_OBJECT_CONFIGURED);
}

void FSMInterface::exit(events *eventData, SystemState& system_state){

  // stop the arm and shutdown communication
  if (in_comm_with_hw == true) {
    if (system_state.ft_sensor.present) {
      std::cout << "Stopping ft-sensor..." << std::endl;
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

// decision of which behavior to execute based on events and arm state
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