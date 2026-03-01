/*
 * This is an auto-generated file. Do not edit it directly.
 * 
 * FSM: kinova_single_arm_demo
 * FSM Description: Example of a simple FSM for collaborative boax lifting application with a single Kinova arm.
 *
 * -----------------------------------------------------
 * Usage example:
 * -----------------------------------------------------

#include "kinova_single_arm_demo.fsm.hpp"

struct user_data {

};

void yyyy_behavior(struct user_data *userData, struct events *eventData) {
    // ... do something

    produce_event(eventData, E_ZZZZ);
}

void fsm_behavior(struct events *eventData, struct user_data *userData) {
    if (consume_event(eventData, E_XXXX)) {
        yyyy_behavior(userData, eventData);
    }
    ...
}

int main() {

    struct user_data userData = {};

    while (true) {
        produce_event(fsm.eventData, E_STEP);

        // run state machine, event loop
        fsm_behavior(fsm.eventData, &userData);
        fsm_step_nbx(&fsm);
        reconfig_event_buffers(&eventData);
    }

    return 0;
}

 * -----------------------------------------------------
 */

#ifndef KINOVA_SINGLE_ARM_DEMO_FSM_HPP
#define KINOVA_SINGLE_ARM_DEMO_FSM_HPP

#include "coord2b/functions/fsm.h"
#include "coord2b/functions/event_loop.h"

// sm states
enum e_states {
    S_START = 0,
    S_CONFIGURE,
    S_IDLE,
    S_EXECUTE,
    S_EXIT,
    S_M_TOUCH_TABLE,
    S_M_SLIDE_ALONG_TABLE,
    S_M_GRASP_OBJECT,
    S_M_COLLABORATE,
    S_M_RELEASE_OBJECT,
    NUM_STATES
};

// sm events
enum e_events {
    E_STEP = 0,
    E_CONFIGURED,
    E_IDLE_EXECUTE,
    E_IDLE_EXIT,
    E_EXECUTE_EXIT,
    E_EXECUTE_IDLE,
    E_M_TOUCH_TABLE_CONFIG,
    E_M_TOUCH_TABLE_CONFIGURED,
    E_M_SLIDE_ALONG_TABLE_CONFIG,
    E_M_SLIDE_ALONG_TABLE_CONFIGURED,
    E_M_GRASP_OBJECT_CONFIG,
    E_M_GRASP_OBJECT_CONFIGURED,
    E_M_COLLABORATE_CONFIG,
    E_M_COLLABORATE_CONFIGURED,
    E_M_RELEASE_OBJECT_CONFIG,
    E_M_RELEASE_OBJECT_CONFIGURED,
    E_CONFIGURE_EXIT,
    E_ENTER_CONFIGURE,
    E_ENTER_IDLE,
    E_ENTER_EXECUTE,
    E_ENTER_EXIT,
    E_ENTER_M_TOUCH_TABLE,
    E_ENTER_M_SLIDE_ALONG_TABLE,
    E_ENTER_M_GRASP_OBJECT,
    E_ENTER_M_COLLABORATE,
    E_ENTER_M_RELEASE_OBJECT,
    NUM_EVENTS
};

// sm transitions
enum e_transitions {
    T_START_CONFIGURE = 0,
    T_CONFIGURE_IDLE,
    T_IDLE_IDLE,
    T_IDLE_EXECUTE,
    T_IDLE_EXIT,
    T_EXECUTE_EXECUTE,
    T_EXECUTE_IDLE,
    T_EXECUTE_EXIT,
    T_EXECUTE_M_TOUCH_TABLE,
    T_M_TOUCH_TABLE_EXECUTE,
    T_EXECUTE_M_SLIDE_ALONG_TABLE,
    T_M_SLIDE_ALONG_TABLE_EXECUTE,
    T_EXECUTE_M_GRASP_OBJECT,
    T_M_GRASP_OBJECT_EXECUTE,
    T_EXECUTE_M_COLLABORATE,
    T_M_COLLABORATE_EXECUTE,
    T_EXECUTE_M_RELEASE_OBJECT,
    T_M_RELEASE_OBJECT_EXECUTE,
    T_CONFIGURE_EXIT,
    NUM_TRANSITIONS
};

// sm reactions
enum e_reactions {
    R_E_CONFIGURE = 0,
    R_E_CONFIGURE_EXIT,
    R_E_IDLE_EXECUTE,
    R_E_EXECUTE_IDLE,
    R_E_IDLE_EXIT,
    R_E_EXECUTE_EXIT,
    R_E_EXECUTE_M_TOUCH_TABLE,
    R_E_EXECUTE_M_SLIDE_ALONG_TABLE,
    R_E_EXECUTE_M_GRASP_OBJECT,
    R_E_EXECUTE_M_COLLABORATE,
    R_E_EXECUTE_M_RELEASE_OBJECT,
    R_E_M_TOUCH_TABLE_CONFIGURED,
    R_E_M_SLIDE_ALONG_TABLE_CONFIGURED,
    R_E_M_GRASP_OBJECT_CONFIGURED,
    R_E_M_COLLABORATE_CONFIGURED,
    R_E_M_RELEASE_OBJECT_CONFIGURED,
    R_E_STEP_START,
    R_E_STEP_IDLE,
    R_E_STEP_EXECUTE,
    NUM_REACTIONS
};

// sm states
inline struct state states[NUM_STATES] = {
    {.name = "S_start"}, 
    {.name = "S_configure"}, 
    {.name = "S_idle"}, 
    {.name = "S_execute"}, 
    {.name = "S_exit"}, 
    {.name = "S_m_touch_table"}, 
    {.name = "S_m_slide_along_table"}, 
    {.name = "S_m_grasp_object"}, 
    {.name = "S_m_collaborate"}, 
    {.name = "S_m_release_object"} 
};

// sm transition table
inline struct transition transitions[NUM_TRANSITIONS] = {
    {
        .startStateIndex = S_START,
        .endStateIndex = S_CONFIGURE,
    }, 
    {
        .startStateIndex = S_CONFIGURE,
        .endStateIndex = S_IDLE,
    }, 
    {
        .startStateIndex = S_IDLE,
        .endStateIndex = S_IDLE,
    }, 
    {
        .startStateIndex = S_IDLE,
        .endStateIndex = S_EXECUTE,
    }, 
    {
        .startStateIndex = S_IDLE,
        .endStateIndex = S_EXIT,
    }, 
    {
        .startStateIndex = S_EXECUTE,
        .endStateIndex = S_EXECUTE,
    }, 
    {
        .startStateIndex = S_EXECUTE,
        .endStateIndex = S_IDLE,
    }, 
    {
        .startStateIndex = S_EXECUTE,
        .endStateIndex = S_EXIT,
    }, 
    {
        .startStateIndex = S_EXECUTE,
        .endStateIndex = S_M_TOUCH_TABLE,
    }, 
    {
        .startStateIndex = S_M_TOUCH_TABLE,
        .endStateIndex = S_EXECUTE,
    }, 
    {
        .startStateIndex = S_EXECUTE,
        .endStateIndex = S_M_SLIDE_ALONG_TABLE,
    }, 
    {
        .startStateIndex = S_M_SLIDE_ALONG_TABLE,
        .endStateIndex = S_EXECUTE,
    }, 
    {
        .startStateIndex = S_EXECUTE,
        .endStateIndex = S_M_GRASP_OBJECT,
    }, 
    {
        .startStateIndex = S_M_GRASP_OBJECT,
        .endStateIndex = S_EXECUTE,
    }, 
    {
        .startStateIndex = S_EXECUTE,
        .endStateIndex = S_M_COLLABORATE,
    }, 
    {
        .startStateIndex = S_M_COLLABORATE,
        .endStateIndex = S_EXECUTE,
    }, 
    {
        .startStateIndex = S_EXECUTE,
        .endStateIndex = S_M_RELEASE_OBJECT,
    }, 
    {
        .startStateIndex = S_M_RELEASE_OBJECT,
        .endStateIndex = S_EXECUTE,
    }, 
    {
        .startStateIndex = S_CONFIGURE,
        .endStateIndex = S_EXIT,
    } 
};

// sm reaction table
inline struct event_reaction reactions[NUM_REACTIONS] = {
    {
        .conditionEventIndex = E_CONFIGURED,
        .transitionIndex = T_CONFIGURE_IDLE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_IDLE 
        },
    }, 
    {
        .conditionEventIndex = E_CONFIGURE_EXIT,
        .transitionIndex = T_CONFIGURE_EXIT,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXIT 
        },
    }, 
    {
        .conditionEventIndex = E_IDLE_EXECUTE,
        .transitionIndex = T_IDLE_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXECUTE 
        },
    }, 
    {
        .conditionEventIndex = E_EXECUTE_IDLE,
        .transitionIndex = T_EXECUTE_IDLE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_IDLE 
        },
    }, 
    {
        .conditionEventIndex = E_IDLE_EXIT,
        .transitionIndex = T_IDLE_EXIT,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXIT 
        },
    }, 
    {
        .conditionEventIndex = E_EXECUTE_EXIT,
        .transitionIndex = T_EXECUTE_EXIT,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXIT 
        },
    }, 
    {
        .conditionEventIndex = E_M_TOUCH_TABLE_CONFIG,
        .transitionIndex = T_EXECUTE_M_TOUCH_TABLE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_M_TOUCH_TABLE 
        },
    }, 
    {
        .conditionEventIndex = E_M_SLIDE_ALONG_TABLE_CONFIG,
        .transitionIndex = T_EXECUTE_M_SLIDE_ALONG_TABLE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_M_SLIDE_ALONG_TABLE 
        },
    }, 
    {
        .conditionEventIndex = E_M_GRASP_OBJECT_CONFIG,
        .transitionIndex = T_EXECUTE_M_GRASP_OBJECT,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_M_GRASP_OBJECT 
        },
    }, 
    {
        .conditionEventIndex = E_M_COLLABORATE_CONFIG,
        .transitionIndex = T_EXECUTE_M_COLLABORATE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_M_COLLABORATE 
        },
    }, 
    {
        .conditionEventIndex = E_M_RELEASE_OBJECT_CONFIG,
        .transitionIndex = T_EXECUTE_M_RELEASE_OBJECT,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_M_RELEASE_OBJECT 
        },
    }, 
    {
        .conditionEventIndex = E_M_TOUCH_TABLE_CONFIGURED,
        .transitionIndex = T_M_TOUCH_TABLE_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXECUTE 
        },
    }, 
    {
        .conditionEventIndex = E_M_SLIDE_ALONG_TABLE_CONFIGURED,
        .transitionIndex = T_M_SLIDE_ALONG_TABLE_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXECUTE 
        },
    }, 
    {
        .conditionEventIndex = E_M_GRASP_OBJECT_CONFIGURED,
        .transitionIndex = T_M_GRASP_OBJECT_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXECUTE 
        },
    }, 
    {
        .conditionEventIndex = E_M_COLLABORATE_CONFIGURED,
        .transitionIndex = T_M_COLLABORATE_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXECUTE 
        },
    }, 
    {
        .conditionEventIndex = E_M_RELEASE_OBJECT_CONFIGURED,
        .transitionIndex = T_M_RELEASE_OBJECT_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXECUTE 
        },
    }, 
    {
        .conditionEventIndex = E_STEP,
        .transitionIndex = T_START_CONFIGURE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_CONFIGURE 
        },
    }, 
    {
        .conditionEventIndex = E_STEP,
        .transitionIndex = T_IDLE_IDLE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_IDLE 
        },
    }, 
    {
        .conditionEventIndex = E_STEP,
        .transitionIndex = T_EXECUTE_EXECUTE,
        .numFiredEvents = 1,
        .firedEventIndices = new unsigned int[1]{
            E_ENTER_EXECUTE 
        },
    } 
};

// sm event data
inline struct events eventData = {
    .numEvents = NUM_EVENTS,
    .currentEvents = new _Bool[NUM_EVENTS]{false},
    .futureEvents = new _Bool[NUM_EVENTS]{false},
};

// sm fsm struct
inline struct fsm_nbx fsm = {
    .numReactions = NUM_REACTIONS,
    .numTransitions = NUM_TRANSITIONS,
    .numStates = NUM_STATES,

    .states = states,
    .startStateIndex = S_START,
    .endStateIndex = S_EXIT,
    .currentStateIndex = S_START,

    .eventData = &eventData,
    .reactions = reactions,
    .transitions = transitions,
};

#endif // KINOVA_SINGLE_ARM_DEMO_FSM_HPP