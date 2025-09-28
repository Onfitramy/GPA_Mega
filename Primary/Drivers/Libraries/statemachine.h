#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "main.h"

/* --- Define all possible flight states --- */
typedef enum {
    STATE_FLIGHT_ABORT,             // abort encountered
    STATE_FLIGHT_INIT,              // initialization
    STATE_FLIGHT_GNC_ALIGN,         // ekf filters converging
    STATE_FLIGHT_CHECKOUTS,         // checkouts running
    STATE_FLIGHT_ARMED,             // armed, awaiting liftoff
    STATE_FLIGHT_BURN,              // motor burn phase
    STATE_FLIGHT_COAST,             // ascending coast
    STATE_FLIGHT_DESCEND_UNBRAKED,  // unbraked descend
    STATE_FLIGHT_DESCEND_DROGUE,    // descend under drogue
    STATE_FLIGHT_DESCEND_MAIN,      // descend under main
    STATE_FLIGHT_LANDED,            // touched down

    STATE_FLIGHT_MAX
} flight_state_t;

/* --- Define all possible flight events --- */
typedef enum {
    EVENT_FLIGHT_GNSS_FIX,
    EVENT_FLIGHT_FILTER_CONVERGED,
    EVENT_FLIGHT_CHECKOUTS_COMPLETE,
    EVENT_FLIGHT_LAUNCH_DETECTED,
    EVENT_FLIGHT_BURNOUT_DETECTED,
    EVENT_FLIGHT_APOGEE_DETECTED,
    EVENT_FLIGHT_DROGUE_DEPLOY,
    EVENT_FLIGHT_MAIN_DEPLOY,
    EVENT_FLIGHT_TOUCHDOWN,

    EVENT_FLIGHT_MAX
} flight_event_t;

typedef struct {
    flight_state_t currentFlightState;  // Holds the current flight state
    uint32_t timestamp_us;              // Holds time of entering current state
} StateMachine_t;

void StateMachine_Init(StateMachine_t *sm, flight_state_t initialState);
void StateMachine_Dispatch(StateMachine_t *sm, flight_event_t event);
void StateMachine_DoActions(StateMachine_t *sm);

#endif // STATEMACHINE_H