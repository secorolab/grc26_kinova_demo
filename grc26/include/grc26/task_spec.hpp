#ifndef TASK_SPEC_HPP
#define TASK_SPEC_HPP

#include <stdint.h>

constexpr int MAX_CONSTRAINTS = 10;
constexpr int NUM_JOINTS = 7;

enum class LinearMode : uint8_t
{
    None = 0,
    Velocity,
    Position,
    Force
};

enum class OrientationMode : uint8_t
{
    None = 0,
    Position,
    Velocity,
    Torque
};

enum class CompareOp : uint8_t
{
    GreaterEqual = 0,
    LessEqual,
    EqualWithinTolerance
};

enum class ConstraintType : uint8_t
{
    Position = 0,
    Velocity,
    Force,
    Torque
};

enum class LogicOp : uint8_t
{
    And = 0,
    Or
};

struct Constraint
{
    ConstraintType type;          // position, velocity, force, or torque
    int            axis;          // 0=x/roll,1=y/pitch,2=z/yaw, 3=gripper
    CompareOp      op;            // >= or <= or == within tolerance
    double         value;         // threshold
    double tolerance = 0.01;      // used if op == EqualWithinTolerance
};

struct PostCondition
{
    bool       available       = false;
    int        num_constraints = 0;
    LogicOp    logic           = LogicOp::And;   // And / Or
    Constraint constraints[MAX_CONSTRAINTS];
};

struct EELinearCommand
{
    bool enabled       = false;
    LinearMode mode[3];

    double velocity[3] = {0};
    double position[3] = {0};
    double force[3]    = {0};

    double vel_threshold = 0.02;  // used for Position mode
};


struct OrientationCommand
{
    bool enabled     = false;
    OrientationMode mode;     // roll pitch yaw
    
    double rpy[3]    = {0};   // in radians; used if Position
    double torque[3] = {0};   // used if Torque
    double ang_vel[3] = {0};   // used if Velocity
    
    int segment_index;
};

struct JointPositionCommand
{
    bool enabled = false;
    double position[NUM_JOINTS] = {0};
};

struct LinkLinearForceCommand
{
    bool   enabled  = false;
    int    segment_index;
    double force[3] = {0};
};

struct GripperCommand
{
    bool   enabled = false;
    float position;
};

struct CollaborateSpec
{
    bool enabled = false;
    double magnification_factor     = 1.0; // how much to amplify human forces for collaboration
    double external_force_deadband  = 7.0; // forces below this threshold are considered noise and ignored
    double f_ext_saturation_limit = 20.0; // maximum external force to consider for collaboration
};

struct TaskSpec
{
    EELinearCommand        ee_linear;
    OrientationCommand     orientation;
    CollaborateSpec        collaborate_spec;
    JointPositionCommand   joint_position;
    LinkLinearForceCommand link_force;
    GripperCommand         gripper;
    PostCondition          post_condition;
    bool forearm_yaw_control_enabled = false; // if true, will control forearm yaw to be constant throughout the task
    std::string controller_config_path = "controller_gains.yaml";

    void resetDefault()
    {
        *this = TaskSpec{};
    }
};

#endif // TASK_SPEC_HPP