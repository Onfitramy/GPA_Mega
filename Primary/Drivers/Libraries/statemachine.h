#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "main.h"

#define MAX_SELFTEST_TRIES 10

/* --- Define minimum Event delay times in ms --- */
#define MIN_DELAY_UNTIL_FILTER_CONVERGED    1000
#define MIN_DELAY_UNTIL_CHECKOUTS_COMPLETE  1000
#define MIN_DELAY_UNTIL_LAUNCH_DETECTED     500
#define MIN_DELAY_UNTIL_BURNOUT_DETECTED    4000
#define MIN_DELAY_UNTIL_APOGEE_DETECTED     15000
#define MIN_DELAY_UNTIL_TOUCHDOWN           15000

/* --- Define maximum Event delay times in ms --- */
#define MAX_DELAY_UNTIL_BURNOUT_DETECTED    6000
#define MAX_DELAY_UNTIL_APOGEE_DETECTED     25000
#define MAX_DELAY_UNTIL_TOUCHDOWN_BAD       18000
#define MAX_DELAY_UNTIL_TOUCHDOWN_DROGUE    30000
#define MAX_DELAY_UNTIL_TOUCHDOWN_MAIN      60000

/* --- Define all possible flight states --- */
typedef enum {
    STATE_FLIGHT_ABORT,             // abort encountered
    STATE_FLIGHT_STARTUP,           // startup
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
    EVENT_FLIGHT_ABORT,
    EVENT_FLIGHT_STARTUP_COMPLETE,
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

/* --- Extern variables used in the state machine Entry, Do and Exit functions --- */
extern StateMachine_t flight_sm;

extern GPA_Mega gpa_mega;
extern StatusPayload_t status_data;
extern uint8_t selftest_tries;

extern TIM_HandleTypeDef htim7;
extern uint32_t tim7_target_ms;

/* --- Function declarations --- */
void StateMachine_Init(StateMachine_t *sm, flight_state_t initialState);
void StateMachine_Dispatch(StateMachine_t *sm, flight_event_t event);
void StateMachine_ForceState(StateMachine_t *sm, flight_state_t newState);
void StateMachine_DoActions(StateMachine_t *sm, uint16_t freq);

#endif // STATEMACHINE_H