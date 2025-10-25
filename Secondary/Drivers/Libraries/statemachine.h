#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "main.h"
#include "PowerUnit.h"

/* --- Define minimum Event delay times in ms --- */
// #define MIN_DELAY_UNTIL_FILTER_CONVERGED


/* --- Define maximum Event delay times in ms --- */
// #define MAX_DELAY_UNTIL_


/* --- Define all possible states --- */
typedef enum {
    STATE_FAULT,
    STATE_STARTUP,
    STATE_INIT,
    STATE_OPERATIONAL,
    STATE_BATTERY_LOW,
    STATE_BATTERY_CRITICAL,

    STATE_MAX
} sm_state_t;

/* --- Define all possible events --- */
typedef enum {
    EVENT_OVERHEAT,             // battery overheat detected
    EVENT_BATTERY_OK,
    EVENT_BATTERY_LOW,
    EVENT_BATTERY_CRITICAL,
    EVENT_FAULT_CLEARED,        // fault condition cleared
    EVENT_STARTUP_COMPLETE,     // startup sequence complete
    EVENT_INITIALIZED,
    EVENT_FAULT,

    EVENT_MAX
} sm_event_t;

typedef struct {
    sm_state_t currentState;  // Holds the current state
    uint32_t timestamp_us;    // Holds time of entering current state
} StateMachine_t;

typedef sm_state_t (*StateHandler_t)(sm_event_t event);
typedef void (*StateEntry_t)(StateMachine_t *sm);
typedef void (*StateDo_t)(StateMachine_t *sm, uint16_t freq);
typedef void (*StateExit_t)(StateMachine_t *sm);

extern StateMachine_t pu_sm;
/* --- Extern variables used in the state machine Entry, Do and Exit functions --- */
extern health_t health;


extern TIM_HandleTypeDef htim7;
extern uint32_t tim7_target_ms;

/* --- Function declarations --- */
void StateMachine_Init(StateMachine_t *sm, sm_state_t initialState);
void StateMachine_Dispatch(StateMachine_t *sm, sm_event_t event);
void StateMachine_ForceState(StateMachine_t *sm, sm_state_t newState);
void StateMachine_DoActions(StateMachine_t *sm, uint16_t freq);

#endif // STATEMACHINE_H