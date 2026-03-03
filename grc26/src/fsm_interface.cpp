#include "grc26/fsm_interface.hpp"

FSMInterface::FSMInterface(SystemState& system_state,
                          robif2b_kinova_gen3_nbx& rob, 
                          robif2b_kg3_robotiq_gripper_nbx& gripper,
                          robif2b_robotiq_ft_nbx& ft_sensor,
                          TaskStatusData& status)
  : system_state(system_state), rob(rob), gripper(gripper), ft_sensor(ft_sensor), 
  task_status(status), in_comm_with_hw(false)
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
  // TODO: check if controllers should be configured
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
  fkSolverPos = std::make_unique<KDL::ChainFkSolverPos_recursive>(model_->chain());
  fkSolverVel = std::make_unique<KDL::ChainFkSolverVel_recursive>(model_->chain());

  if (!solver_->initialize(6)) {
    printf("Solver initialization failed\n");
    produce_event(eventData, E_CONFIGURE_EXIT);
    return;
  }

  // initialise solver state interface
  solver_state_interface_ =
    std::make_unique<SolverStateInterface>(*solver_);

  // establish communication with arm
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

  in_comm_with_hw = true;
  // emit event to transition to idle state after configuration
  produce_event(eventData, E_CONFIGURED);
}

void FSMInterface::idle(events *eventData, const SystemState& system_state){
  robif2b_kinova_gen3_update(&rob);
  // set controller state

  task_setpoint.ee_linear.enabled = true;
  task_setpoint.ee_linear.mode[0] = LinearMode::Position;
  task_setpoint.ee_linear.mode[1] = LinearMode::Position;
  task_setpoint.ee_linear.mode[2] = LinearMode::Position;
  task_setpoint.ee_linear.position[0] = system_state.arm.pose_ee_BL.p[0]; // m
  task_setpoint.ee_linear.position[1] = system_state.arm.pose_ee_BL.p[1]; // m
  task_setpoint.ee_linear.position[2] = system_state.arm.pose_ee_BL.p[2]; // m

  task_setpoint.orientation.enabled = true;
  task_setpoint.orientation.mode[0] = OrientationMode::Position;
  task_setpoint.orientation.mode[1] = OrientationMode::Position;
  task_setpoint.orientation.mode[2] = OrientationMode::Position;

  double roll, pitch, yaw;
  system_state.arm.pose_ee_BL.M.GetRPY(roll, pitch, yaw);

  task_setpoint.orientation.rpy[0] = roll; // roll
  task_setpoint.orientation.rpy[1] = pitch; // pitch
  task_setpoint.orientation.rpy[2] = yaw; // yaw

}

void FSMInterface::execute(events *eventData, SystemState& system_state){

  // solve for control commands

  // update control commands

  // update state variable in cartesian space

  solver_state_interface_->toSolver(system_state);
  solver_->computeTorques();
  solver_state_interface_->fromSolver(system_state);

  // send control commands to robot
  robif2b_kinova_gen3_update(&rob);
  fkSolverPos->JntToCart(solver_->q(), system_state.arm.pose_ee_BL);
  KDL::JntArrayVel qvel(solver_->q(), solver_->qd());
  KDL::FrameVel frame_vel;
  fkSolverVel->JntToCart(qvel, frame_vel);
  system_state.arm.twist_ee_BL = frame_vel.GetTwist();
}

void FSMInterface::touch_table_behavior_config(events *eventData, SystemState& system_state){
  
  task_setpoint.ee_linear.enabled = true;
  task_setpoint.ee_linear.mode[0] = LinearMode::Velocity;
  task_setpoint.ee_linear.mode[1] = LinearMode::Velocity;
  task_setpoint.ee_linear.mode[2] = LinearMode::Velocity;
  task_setpoint.ee_linear.velocity[0] = 0.0;   // m/s
  task_setpoint.ee_linear.velocity[1] = 0.0;   // m/s
  task_setpoint.ee_linear.velocity[2] = -0.01; // m/s

  task_setpoint.orientation.enabled = true;
  task_setpoint.orientation.mode[0] = OrientationMode::Position;
  task_setpoint.orientation.mode[1] = OrientationMode::Position;
  task_setpoint.orientation.mode[2] = OrientationMode::Position;
  task_setpoint.orientation.rpy[0] = 0.0; // roll
  task_setpoint.orientation.rpy[1] = 0.0; // pitch
  task_setpoint.orientation.rpy[2] = 0.0; // yaw

  task_setpoint.post_condition.available = true;
  task_setpoint.post_condition.num_constraints = 2;
  task_setpoint.post_condition.logic = LogicOp::And;

  task_setpoint.post_condition.constraints[0].type = ConstraintType::Position;
  task_setpoint.post_condition.constraints[0].axis = 2; // z-axis
  task_setpoint.post_condition.constraints[0].op = CompareOp::LessEqual;
  task_setpoint.post_condition.constraints[0].value = 0.02; // m

  task_setpoint.post_condition.constraints[1].type = ConstraintType::Velocity;
  task_setpoint.post_condition.constraints[1].axis = 2; // z-axis
  task_setpoint.post_condition.constraints[1].op = CompareOp::LessEqual;
  task_setpoint.post_condition.constraints[1].value = 0.01; // m/s

  produce_event(eventData, E_M_SLIDE_ALONG_TABLE_CONFIGURED);
}

void FSMInterface::slide_on_table_behavior_config(events *eventData, SystemState& system_state){

  task_setpoint.ee_linear.enabled = true;
  task_setpoint.ee_linear.mode[0] = LinearMode::Velocity;
  task_setpoint.ee_linear.mode[1] = LinearMode::Velocity;
  task_setpoint.ee_linear.mode[2] = LinearMode::Force;
  task_setpoint.ee_linear.velocity[0] = 0.0;    // m/s
  task_setpoint.ee_linear.velocity[1] = 0.01;   // m/s
  task_setpoint.ee_linear.force[2] = 5.0;       // N

  task_setpoint.orientation.enabled = true;
  task_setpoint.orientation.mode[0] = OrientationMode::Position;
  task_setpoint.orientation.mode[1] = OrientationMode::Position;
  task_setpoint.orientation.mode[2] = OrientationMode::Position;
  task_setpoint.orientation.rpy[0] = 0.0; // roll
  task_setpoint.orientation.rpy[1] = 0.0; // pitch
  task_setpoint.orientation.rpy[2] = 0.0; // yaw

  task_setpoint.post_condition.available = true;
  task_setpoint.post_condition.num_constraints = 2;
  task_setpoint.post_condition.logic = LogicOp::And;

  task_setpoint.post_condition.constraints[0].type = ConstraintType::Position;
  task_setpoint.post_condition.constraints[0].axis = 2; // z-axis
  task_setpoint.post_condition.constraints[0].op = CompareOp::LessEqual;
  task_setpoint.post_condition.constraints[0].value = 0.02; // m

  task_setpoint.post_condition.constraints[1].type = ConstraintType::Velocity;
  task_setpoint.post_condition.constraints[1].axis = 2; // z-axis
  task_setpoint.post_condition.constraints[1].op = CompareOp::LessEqual;
  task_setpoint.post_condition.constraints[1].value = 0.01; // m/s

  produce_event(eventData, E_M_TOUCH_TABLE_CONFIGURED);
}

void FSMInterface::grasp_object_behavior_config(events *eventData, SystemState& system_state){

  task_setpoint.gripper.enabled = true;
  task_setpoint.gripper.position = 1.0; // fully closed

  task_setpoint.post_condition.available = true;
  task_setpoint.post_condition.num_constraints = 1;

  task_setpoint.post_condition.constraints[0].type = ConstraintType::Position;
  task_setpoint.post_condition.constraints[0].axis = 3; // gripper axis
  task_setpoint.post_condition.constraints[0].op = CompareOp::GreaterEqual;
  task_setpoint.post_condition.constraints[0].value = 0.95; // fully closed

  produce_event(eventData, E_M_GRASP_OBJECT_CONFIGURED);
}

void FSMInterface::collaborate_behavior_config(events *eventData, SystemState& system_state){

  // TODO
  
  produce_event(eventData, E_M_COLLABORATE_CONFIGURED);
}

void FSMInterface::release_object_behavior_config(events *eventData, SystemState& system_state){
  task_setpoint.gripper.enabled = true;
  task_setpoint.gripper.position = 0.0; // fully open

  task_setpoint.post_condition.available = true;
  task_setpoint.post_condition.num_constraints = 1;

  task_setpoint.post_condition.constraints[0].type = ConstraintType::Position;
  task_setpoint.post_condition.constraints[0].axis = 3; // gripper axis
  task_setpoint.post_condition.constraints[0].op = CompareOp::LessEqual;
  task_setpoint.post_condition.constraints[0].value = 0.1; // fully closed


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

void FSMInterface::compute_gravity_comp(events *eventData, SystemState& system_state){
  // TODO
}

void FSMInterface::run_fsm(){
  produce_event(&eventData, E_STEP);
  fsm_behavior(&eventData, system_state);
  fsm_step_nbx(&fsm);
  reconfig_event_buffers(&eventData);
};