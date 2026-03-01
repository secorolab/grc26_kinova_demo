
#ifndef FSM_INTERFACE_HPP
#define FSM_INTERFACE_HPP

#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <atomic>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

#include <kdl_parser/kdl_parser.hpp>

#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainjnttojacdotsolver.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"

#include "robif2b/functions/ethercat.h"
#include "robif2b/functions/kinova_gen3.h"

#include "grc26/kinova_single_arm_demo.fsm.hpp"
#include "grc26/pid_controller.hpp"
#include "grc26/cartesian_motion_setpoint.hpp"
#include "robif2b/functions/kinova_gen3.h"
#include "grc26/arm_state.hpp"

#define NUM_JOINTS 7

#define KINOVA_TAU_CMD_LIMIT 30.0


class FSMInterface
{
public:
    explicit FSMInterface(robif2b_kinova_gen3_nbx& rob);
    ~FSMInterface();

    int get_current_state() const;
    bool is_in_comm_with_hw() const;

    // FSM methods
    void configure(events *eventData, ArmState& arm_state);
    void idle(events *eventData, const ArmState& arm_state);
    void execute(events *eventData, ArmState& arm_state);

    void touch_table_behavior_config(events *eventData, ArmState& arm_state);
    void slide_on_table_behavior_config(events *eventData, ArmState& arm_state);
    void grasp_object_behavior_config(events *eventData, ArmState& arm_state);
    void collaborate_behavior_config(events *eventData, ArmState& arm_state);
    void release_object_behavior_config(events *eventData, ArmState& arm_state);

    // decision of which behavior to execute based on events and arm state
    void fsm_behavior(events *eventData, ArmState& arm_state);

    void compute_gravity_comp(events *eventData, ArmState& arm_state);
    void compute_cartesian_ctrl(events *eventData, ArmState& arm_state);
    void exit(events *eventData, ArmState& arm_state);
    void run_fsm();

public:
    bool enable_arm_ctrl = true;

private:
    robif2b_kinova_gen3_nbx& rob;
    CartesianMotionSetpoint sp;
    bool in_comm_with_hw = false;

    // KDL members
    KDL::Tree tree;
    KDL::Chain arm_chain;

    int num_jnts = 0;
    int num_segs = 0;

    KDL::Twist root_acc;
    KDL::JntArray q;
    KDL::JntArray qd;
    KDL::JntArray qdd;
    KDL::JntArray tau_ctrl;

    KDL::Wrenches f_ext;
    KDL::Frame pose_ee;
    KDL::Twist twist_ee;
    KDL::Frame target_pose_ee;

    ArmState arm_state;
};

#endif // FSM_INTERFACE_HPP