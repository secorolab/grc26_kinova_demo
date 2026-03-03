#ifndef TASK_SETPOINT_HPP
#define TASK_SETPOINT_HPP

#include <stdint.h>

constexpr int MAX_CONSTRAINTS = 10;

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
    Torque
};

enum class CompareOp : uint8_t
{
    GreaterEqual = 0,
    LessEqual
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
    ConstraintType type;          // what to check
    int            axis;          // 0=x/roll,1=y/pitch,2=z/yaw, 3=gripper
    int            segment_index; // link to check (EE or other)
    CompareOp      op;            // >= or <=
    double         value;         // threshold
};

struct PostCondition
{
    bool       available = false;
    Constraint constraints[MAX_CONSTRAINTS];
    int        num_constraints = 0;
    LogicOp    logic = LogicOp::And;   // And / Or
};

struct EELinearCommand
{
    bool   enabled = false;
    LinearMode mode[3];     // x y z

    double velocity[3];     // used if Velocity
    double position[3];     // used if Position
    double force[3];        // used if Force

    double vel_threshold;   // used for Position mode
};

struct OrientationCommand
{
    bool   enabled = false;
    OrientationMode mode[3];  // roll pitch yaw

    double rpy[3];            // used if Position
    double torque[3];         // used if Torque

    int segment_index;
};

struct LinkLinearForceCommand
{
    bool   enabled = false;
    int    segment_index;
    double force[3];
};

struct GripperCommand
{
    bool   enabled = false;
    double position;
};

struct TaskSetpoint
{
    EELinearCommand        ee_linear;
    OrientationCommand     orientation;
    LinkLinearForceCommand link_force;
    GripperCommand         gripper;
    PostCondition          post_condition;
};

#endif // TASK_SETPOINT_HPP