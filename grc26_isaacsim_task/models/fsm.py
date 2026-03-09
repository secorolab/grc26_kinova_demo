"""
This is an auto-generated file. Do not edit it directly.

FSM: kinova_single_arm_demo
FSM Description: Example of a simple FSM for collaborative boax lifting application with a single Kinova arm.

Examples:

>>> from coord_dsl.fsm import fsm_step
>>> from coord_dsl.event_loop import reconfig_event_buffers
>>> from fsm_example import create_fsm
>>> fsm = create_fsm()
>>> while True:
...     if fsm.current_state_index == StateID.S_EXIT:
...         print("State machine completed successfully")
...         break
...     fsm_behavior(fsm, ud) # user-defined behaviour with user data
...     fsm_step(fsm)
...     reconfig_event_buffers(fsm.event_data)
"""
from enum import IntEnum, auto
from coord_dsl.event_loop import EventData
from coord_dsl.fsm import FSMData, Transition, EventReaction


# Event IDs
class EventID(IntEnum):
    E_STEP = 0
    E_CONFIGURED = auto()
    E_IDLE_EXECUTE = auto()
    E_IDLE_EXIT = auto()
    E_EXECUTE_EXIT = auto()
    E_EXECUTE_IDLE = auto()
    E_M_HOME_CONFIG = auto()
    E_M_HOME_CONFIGURED = auto()
    E_M_OPEN_GRIPPER_CONFIG = auto()
    E_M_OPEN_GRIPPER_CONFIGURED = auto()
    E_M_TOUCH_TABLE_CONFIG = auto()
    E_M_TOUCH_TABLE_CONFIGURED = auto()
    E_M_SLIDE_ALONG_TABLE_CONFIG = auto()
    E_M_SLIDE_ALONG_TABLE_CONFIGURED = auto()
    E_M_GRASP_OBJECT_CONFIG = auto()
    E_M_GRASP_OBJECT_CONFIGURED = auto()
    E_M_COLLABORATE_CONFIG = auto()
    E_M_COLLABORATE_CONFIGURED = auto()
    E_M_RELEASE_OBJECT_CONFIG = auto()
    E_M_RELEASE_OBJECT_CONFIGURED = auto()
    E_M_RETRACT_ARM_CONFIG = auto()
    E_M_RETRACT_ARM_CONFIGURED = auto()
    E_M_RETURN_HOME_CONFIG = auto()
    E_M_RETURN_HOME_CONFIGURED = auto()


# State IDs
class StateID(IntEnum):
    S_START = 0
    S_CONFIGURE = auto()
    S_IDLE = auto()
    S_EXECUTE = auto()
    S_EXIT = auto()
    S_M_HOME = auto()
    S_M_OPEN_GRIPPER = auto()
    S_M_TOUCH_TABLE = auto()
    S_M_SLIDE_ALONG_TABLE = auto()
    S_M_GRASP_OBJECT = auto()
    S_M_COLLABORATE = auto()
    S_M_RELEASE_OBJECT = auto()
    S_M_RETRACT_ARM = auto()
    S_M_RETURN_HOME = auto()


# Transition IDs
class TransitionID(IntEnum):
    T_START_CONFIGURE = 0
    T_CONFIGURE_IDLE = auto()
    T_IDLE_IDLE = auto()
    T_IDLE_EXECUTE = auto()
    T_IDLE_EXIT = auto()
    T_EXECUTE_EXECUTE = auto()
    T_EXECUTE_IDLE = auto()
    T_EXECUTE_EXIT = auto()
    T_IDLE_M_HOME = auto()
    T_M_HOME_EXECUTE = auto()
    T_EXECUTE_M_OPEN_GRIPPER = auto()
    T_M_OPEN_GRIPPER_EXECUTE = auto()
    T_EXECUTE_M_TOUCH_TABLE = auto()
    T_M_TOUCH_TABLE_EXECUTE = auto()
    T_EXECUTE_M_SLIDE_ALONG_TABLE = auto()
    T_M_SLIDE_ALONG_TABLE_EXECUTE = auto()
    T_EXECUTE_M_GRASP_OBJECT = auto()
    T_M_GRASP_OBJECT_EXECUTE = auto()
    T_EXECUTE_M_COLLABORATE = auto()
    T_M_COLLABORATE_EXECUTE = auto()
    T_EXECUTE_M_RELEASE_OBJECT = auto()
    T_M_RELEASE_OBJECT_EXECUTE = auto()
    T_EXECUTE_M_RETRACT_ARM = auto()
    T_M_RETRACT_ARM_EXECUTE = auto()
    T_M_EXECUTE_RETURN_HOME = auto()
    T_M_RETURN_HOME_EXECUTE = auto()


# Event reaction IDs
class ReactionID(IntEnum):
    R_E_CONFIGURE = 0
    R_E_IDLE_EXECUTE = auto()
    R_E_EXECUTE_IDLE = auto()
    R_E_IDLE_EXIT = auto()
    R_E_EXECUTE_EXIT = auto()
    R_E_M_HOME_CONFIG = auto()
    R_E_M_HOME_CONFIGURED = auto()
    R_E_OPEN_GRIPPER_CONFIG = auto()
    R_E_OPEN_GRIPPER_CONFIGURED = auto()
    R_E_EXECUTE_M_TOUCH_TABLE = auto()
    R_E_EXECUTE_M_SLIDE_ALONG_TABLE = auto()
    R_E_EXECUTE_M_GRASP_OBJECT = auto()
    R_E_EXECUTE_M_COLLABORATE = auto()
    R_E_M_TOUCH_TABLE_CONFIGURED = auto()
    R_E_M_SLIDE_ALONG_TABLE_CONFIGURED = auto()
    R_E_M_GRASP_OBJECT_CONFIGURED = auto()
    R_E_M_COLLABORATE_CONFIGURED = auto()
    R_E_EXECUTE_M_RELEASE_OBJECT = auto()
    R_E_M_RELEASE_OBJECT_CONFIGURED = auto()
    R_E_EXECUTE_M_RETRACT_ARM = auto()
    R_E_M_RETRACT_ARM_CONFIGURED = auto()
    R_E_EXECUTE_RETURN_HOME = auto()
    R_E_RETURN_HOME_CONFIGURED = auto()
    R_E_STEP_START = auto()
    R_E_STEP_IDLE = auto()
    R_E_STEP_EXECUTE = auto()


def create_fsm() -> FSMData:
    """Creates the FSM data structure."""
    # Transitions
    trans_dict = {
        TransitionID.T_START_CONFIGURE: Transition(StateID.S_START, StateID.S_CONFIGURE),
        TransitionID.T_CONFIGURE_IDLE: Transition(StateID.S_CONFIGURE, StateID.S_IDLE),
        TransitionID.T_IDLE_IDLE: Transition(StateID.S_IDLE, StateID.S_IDLE),
        TransitionID.T_IDLE_EXECUTE: Transition(StateID.S_IDLE, StateID.S_EXECUTE),
        TransitionID.T_IDLE_EXIT: Transition(StateID.S_IDLE, StateID.S_EXIT),
        TransitionID.T_EXECUTE_EXECUTE: Transition(StateID.S_EXECUTE, StateID.S_EXECUTE),
        TransitionID.T_EXECUTE_IDLE: Transition(StateID.S_EXECUTE, StateID.S_IDLE),
        TransitionID.T_EXECUTE_EXIT: Transition(StateID.S_EXECUTE, StateID.S_EXIT),
        TransitionID.T_IDLE_M_HOME: Transition(StateID.S_IDLE, StateID.S_M_HOME),
        TransitionID.T_M_HOME_EXECUTE: Transition(StateID.S_M_HOME, StateID.S_EXECUTE),
        TransitionID.T_EXECUTE_M_OPEN_GRIPPER: Transition(StateID.S_EXECUTE, StateID.S_M_OPEN_GRIPPER),
        TransitionID.T_M_OPEN_GRIPPER_EXECUTE: Transition(StateID.S_M_OPEN_GRIPPER, StateID.S_EXECUTE),
        TransitionID.T_EXECUTE_M_TOUCH_TABLE: Transition(StateID.S_EXECUTE, StateID.S_M_TOUCH_TABLE),
        TransitionID.T_M_TOUCH_TABLE_EXECUTE: Transition(StateID.S_M_TOUCH_TABLE, StateID.S_EXECUTE),
        TransitionID.T_EXECUTE_M_SLIDE_ALONG_TABLE: Transition(StateID.S_EXECUTE, StateID.S_M_SLIDE_ALONG_TABLE),
        TransitionID.T_M_SLIDE_ALONG_TABLE_EXECUTE: Transition(StateID.S_M_SLIDE_ALONG_TABLE, StateID.S_EXECUTE),
        TransitionID.T_EXECUTE_M_GRASP_OBJECT: Transition(StateID.S_EXECUTE, StateID.S_M_GRASP_OBJECT),
        TransitionID.T_M_GRASP_OBJECT_EXECUTE: Transition(StateID.S_M_GRASP_OBJECT, StateID.S_EXECUTE),
        TransitionID.T_EXECUTE_M_COLLABORATE: Transition(StateID.S_EXECUTE, StateID.S_M_COLLABORATE),
        TransitionID.T_M_COLLABORATE_EXECUTE: Transition(StateID.S_M_COLLABORATE, StateID.S_EXECUTE),
        TransitionID.T_EXECUTE_M_RELEASE_OBJECT: Transition(StateID.S_EXECUTE, StateID.S_M_RELEASE_OBJECT),
        TransitionID.T_M_RELEASE_OBJECT_EXECUTE: Transition(StateID.S_M_RELEASE_OBJECT, StateID.S_EXECUTE),
        TransitionID.T_EXECUTE_M_RETRACT_ARM: Transition(StateID.S_EXECUTE, StateID.S_M_RETRACT_ARM),
        TransitionID.T_M_RETRACT_ARM_EXECUTE: Transition(StateID.S_M_RETRACT_ARM, StateID.S_EXECUTE),
        TransitionID.T_M_EXECUTE_RETURN_HOME: Transition(StateID.S_EXECUTE, StateID.S_M_HOME),
        TransitionID.T_M_RETURN_HOME_EXECUTE: Transition(StateID.S_M_HOME, StateID.S_EXECUTE),
    }
    trans_list = [trans_dict[i] for i in TransitionID]

    # Event Reactions
    evt_reaction_dict = {
        ReactionID.R_E_CONFIGURE: EventReaction(
            condition_event_index=EventID.E_CONFIGURED,
            transition_index=TransitionID.T_CONFIGURE_IDLE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_IDLE_EXECUTE: EventReaction(
            condition_event_index=EventID.E_IDLE_EXECUTE,
            transition_index=TransitionID.T_IDLE_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_EXECUTE_IDLE: EventReaction(
            condition_event_index=EventID.E_EXECUTE_IDLE,
            transition_index=TransitionID.T_EXECUTE_IDLE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_IDLE_EXIT: EventReaction(
            condition_event_index=EventID.E_IDLE_EXIT,
            transition_index=TransitionID.T_IDLE_EXIT,
            fired_event_indices=[],
        ),
        ReactionID.R_E_EXECUTE_EXIT: EventReaction(
            condition_event_index=EventID.E_EXECUTE_EXIT,
            transition_index=TransitionID.T_EXECUTE_EXIT,
            fired_event_indices=[],
        ),
        ReactionID.R_E_M_HOME_CONFIG: EventReaction(
            condition_event_index=EventID.E_M_HOME_CONFIG,
            transition_index=TransitionID.T_IDLE_M_HOME,
            fired_event_indices=[],
        ),
        ReactionID.R_E_M_HOME_CONFIGURED: EventReaction(
            condition_event_index=EventID.E_M_HOME_CONFIGURED,
            transition_index=TransitionID.T_M_HOME_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_OPEN_GRIPPER_CONFIG: EventReaction(
            condition_event_index=EventID.E_M_OPEN_GRIPPER_CONFIG,
            transition_index=TransitionID.T_EXECUTE_M_OPEN_GRIPPER,
            fired_event_indices=[],
        ),
        ReactionID.R_E_OPEN_GRIPPER_CONFIGURED: EventReaction(
            condition_event_index=EventID.E_M_OPEN_GRIPPER_CONFIGURED,
            transition_index=TransitionID.T_M_OPEN_GRIPPER_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_EXECUTE_M_TOUCH_TABLE: EventReaction(
            condition_event_index=EventID.E_M_TOUCH_TABLE_CONFIG,
            transition_index=TransitionID.T_EXECUTE_M_TOUCH_TABLE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_EXECUTE_M_SLIDE_ALONG_TABLE: EventReaction(
            condition_event_index=EventID.E_M_SLIDE_ALONG_TABLE_CONFIG,
            transition_index=TransitionID.T_EXECUTE_M_SLIDE_ALONG_TABLE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_EXECUTE_M_GRASP_OBJECT: EventReaction(
            condition_event_index=EventID.E_M_GRASP_OBJECT_CONFIG,
            transition_index=TransitionID.T_EXECUTE_M_GRASP_OBJECT,
            fired_event_indices=[],
        ),
        ReactionID.R_E_EXECUTE_M_COLLABORATE: EventReaction(
            condition_event_index=EventID.E_M_COLLABORATE_CONFIG,
            transition_index=TransitionID.T_EXECUTE_M_COLLABORATE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_M_TOUCH_TABLE_CONFIGURED: EventReaction(
            condition_event_index=EventID.E_M_TOUCH_TABLE_CONFIGURED,
            transition_index=TransitionID.T_M_TOUCH_TABLE_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_M_SLIDE_ALONG_TABLE_CONFIGURED: EventReaction(
            condition_event_index=EventID.E_M_SLIDE_ALONG_TABLE_CONFIGURED,
            transition_index=TransitionID.T_M_SLIDE_ALONG_TABLE_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_M_GRASP_OBJECT_CONFIGURED: EventReaction(
            condition_event_index=EventID.E_M_GRASP_OBJECT_CONFIGURED,
            transition_index=TransitionID.T_M_GRASP_OBJECT_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_M_COLLABORATE_CONFIGURED: EventReaction(
            condition_event_index=EventID.E_M_COLLABORATE_CONFIGURED,
            transition_index=TransitionID.T_M_COLLABORATE_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_EXECUTE_M_RELEASE_OBJECT: EventReaction(
            condition_event_index=EventID.E_M_RELEASE_OBJECT_CONFIG,
            transition_index=TransitionID.T_EXECUTE_M_RELEASE_OBJECT,
            fired_event_indices=[],
        ),
        ReactionID.R_E_M_RELEASE_OBJECT_CONFIGURED: EventReaction(
            condition_event_index=EventID.E_M_RELEASE_OBJECT_CONFIGURED,
            transition_index=TransitionID.T_M_RELEASE_OBJECT_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_EXECUTE_M_RETRACT_ARM: EventReaction(
            condition_event_index=EventID.E_M_RETRACT_ARM_CONFIG,
            transition_index=TransitionID.T_EXECUTE_M_RETRACT_ARM,
            fired_event_indices=[],
        ),
        ReactionID.R_E_M_RETRACT_ARM_CONFIGURED: EventReaction(
            condition_event_index=EventID.E_M_RETRACT_ARM_CONFIGURED,
            transition_index=TransitionID.T_M_RETRACT_ARM_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_EXECUTE_RETURN_HOME: EventReaction(
            condition_event_index=EventID.E_M_RETURN_HOME_CONFIG,
            transition_index=TransitionID.T_M_EXECUTE_RETURN_HOME,
            fired_event_indices=[],
        ),
        ReactionID.R_E_RETURN_HOME_CONFIGURED: EventReaction(
            condition_event_index=EventID.E_M_RETURN_HOME_CONFIGURED,
            transition_index=TransitionID.T_M_RETURN_HOME_EXECUTE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_STEP_START: EventReaction(
            condition_event_index=EventID.E_STEP,
            transition_index=TransitionID.T_START_CONFIGURE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_STEP_IDLE: EventReaction(
            condition_event_index=EventID.E_STEP,
            transition_index=TransitionID.T_IDLE_IDLE,
            fired_event_indices=[],
        ),
        ReactionID.R_E_STEP_EXECUTE: EventReaction(
            condition_event_index=EventID.E_STEP,
            transition_index=TransitionID.T_EXECUTE_EXECUTE,
            fired_event_indices=[],
        ),
    }
    evt_reaction_list = [evt_reaction_dict[i] for i in ReactionID]

    # Events
    events = EventData(len(EventID))

    # Return FSM Data
    return FSMData(
        event_data=events,
        num_states=len(StateID),
        start_state_index=StateID.S_START,
        end_state_index=StateID.S_EXIT,
        transitions=trans_list,
        event_reactions=evt_reaction_list,
        current_state_index=StateID.S_START,
    )