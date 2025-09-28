#include "statemachine.h"

static flight_state_t AbortHandler(flight_event_t event);
static flight_state_t InitHandler(flight_event_t event);
static flight_state_t AlignGNCHandler(flight_event_t event);
static flight_state_t CheckoutsHandler(flight_event_t event);
static flight_state_t ArmedHandler(flight_event_t event);
static flight_state_t BurnHandler(flight_event_t event);
static flight_state_t CoastHandler(flight_event_t event);
static flight_state_t UnbrakedDescendHandler(flight_event_t event);
static flight_state_t DrogueDescendHandler(flight_event_t event);
static flight_state_t MainDescendHandler(flight_event_t event);
static flight_state_t LandedHandler(flight_event_t event);

/* --- State handler lookup table --- */
typedef flight_state_t (*StateHandler_t)(flight_event_t event);
static StateHandler_t stateTable[STATE_FLIGHT_MAX] = {
    AbortHandler,
    InitHandler,
    AlignGNCHandler,
    CheckoutsHandler,
    ArmedHandler,
    BurnHandler,
    CoastHandler,
    UnbrakedDescendHandler,
    DrogueDescendHandler,
    MainDescendHandler,
    LandedHandler
};

/* --- Entry actions --- */
static void AbortEntry() {}
static void InitEntry() {}
static void AlignGNCEntry() {}
static void CheckoutsEntry() {}
static void ArmedEntry() {}
static void BurnEntry() {}
static void CoastEntry() {}
static void UnbrakedDescendEntry() {}
static void DrogueDescendEntry() {}
static void MainDescendEntry() {}
static void LandedEntry() {}

/* --- Do actions --- */
static void AbortDo() {}
static void InitDo() {}
static void AlignGNCDo() {}
static void CheckoutsDo() {}
static void ArmedDo() {}
static void BurnDo() {}
static void CoastDo() {}
static void UnbrakedDescendDo() {}
static void DrogueDescendDo() {}
static void MainDescendDo() {}
static void LandedDo() {}

/* --- Exit actions --- */
static void AbortExit() {}
static void InitExit() {}
static void AlignGNCExit() {}
static void CheckoutsExit() {}
static void ArmedExit() {}
static void BurnExit() {}
static void CoastExit() {}
static void UnbrakedDescendExit() {}
static void DrogueDescendExit() {}
static void MainDescendExit() {}
static void LandedExit() {}

/* --- Action handler functions --- */
static void StateEntryHandler(flight_state_t state) {
    switch (state) {
        case STATE_FLIGHT_ABORT:            AbortEntry(); break;
        case STATE_FLIGHT_INIT:             InitEntry(); break;
        case STATE_FLIGHT_GNC_ALIGN:        AlignGNCEntry(); break;
        case STATE_FLIGHT_CHECKOUTS:        CheckoutsEntry(); break;
        case STATE_FLIGHT_ARMED:            ArmedEntry(); break;
        case STATE_FLIGHT_BURN:             BurnEntry(); break;
        case STATE_FLIGHT_COAST:            CoastEntry(); break;
        case STATE_FLIGHT_DESCEND_UNBRAKED: UnbrakedDescendEntry(); break;
        case STATE_FLIGHT_DESCEND_DROGUE:   DrogueDescendEntry(); break;
        case STATE_FLIGHT_DESCEND_MAIN:     MainDescendEntry(); break;
        case STATE_FLIGHT_LANDED:           LandedEntry(); break;
        default: break;
    }
}

static void StateExitHandler(flight_state_t state) {
    switch (state) {
        case STATE_FLIGHT_ABORT:            AbortExit(); break;
        case STATE_FLIGHT_INIT:             InitExit(); break;
        case STATE_FLIGHT_GNC_ALIGN:        AlignGNCExit(); break;
        case STATE_FLIGHT_CHECKOUTS:        CheckoutsExit(); break;
        case STATE_FLIGHT_ARMED:            ArmedExit(); break;
        case STATE_FLIGHT_BURN:             BurnExit(); break;
        case STATE_FLIGHT_COAST:            CoastExit(); break;
        case STATE_FLIGHT_DESCEND_UNBRAKED: UnbrakedDescendExit(); break;
        case STATE_FLIGHT_DESCEND_DROGUE:   DrogueDescendExit(); break;
        case STATE_FLIGHT_DESCEND_MAIN:     MainDescendExit(); break;
        case STATE_FLIGHT_LANDED:           LandedExit(); break;
        default: break;
    }
}

static void StateDoHandler(flight_state_t state) {
    switch (state) {
        case STATE_FLIGHT_ABORT:            AbortDo(); break;
        case STATE_FLIGHT_INIT:             InitDo(); break;
        case STATE_FLIGHT_GNC_ALIGN:        AlignGNCDo(); break;
        case STATE_FLIGHT_CHECKOUTS:        CheckoutsDo(); break;
        case STATE_FLIGHT_ARMED:            ArmedDo(); break;
        case STATE_FLIGHT_BURN:             BurnDo(); break;
        case STATE_FLIGHT_COAST:            CoastDo(); break;
        case STATE_FLIGHT_DESCEND_UNBRAKED: UnbrakedDescendDo(); break;
        case STATE_FLIGHT_DESCEND_DROGUE:   DrogueDescendDo(); break;
        case STATE_FLIGHT_DESCEND_MAIN:     MainDescendDo(); break;
        case STATE_FLIGHT_LANDED:           LandedDo(); break;
        default: break;
    }
}

/* --- User functions --- */
void StateMachine_Init(StateMachine_t *sm, flight_state_t initialState) {
    sm->currentFlightState = initialState;

    // call entry action for initial state
    StateEntryHandler(sm->currentFlightState);
}

void StateMachine_Dispatch(StateMachine_t *sm, flight_event_t event) {
    if (sm->currentFlightState >= STATE_FLIGHT_MAX) return;
    
    // store old flight state
    flight_state_t oldState = sm->currentFlightState;

    // handle event and retrieve new flight state
    flight_state_t newState = stateTable[oldState](event);

    if (newState == oldState || newState >= STATE_FLIGHT_MAX) return;
    
    // exit old state
    StateExitHandler(oldState);

    // update flight state
    sm->currentFlightState = newState;

    // enter new state
    StateEntryHandler(newState);
}

void StateMachine_DoActions(StateMachine_t *sm) {
    // perform standard state actions
    StateDoHandler(sm->currentFlightState);
}

/* --- Event handlers --- */
static flight_state_t AbortHandler(flight_event_t event) {
    switch (event) {
        default: return STATE_FLIGHT_ABORT;
    }
}

static flight_state_t InitHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_GNSS_FIX:             return STATE_FLIGHT_GNC_ALIGN;
        default: return STATE_FLIGHT_INIT;
    }
}

static flight_state_t AlignGNCHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_FILTER_CONVERGED:     return STATE_FLIGHT_CHECKOUTS;
        default: return STATE_FLIGHT_GNC_ALIGN;
    }
}

static flight_state_t CheckoutsHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_CHECKOUTS_COMPLETE:   return STATE_FLIGHT_ARMED;
        default: return STATE_FLIGHT_CHECKOUTS;
    }
}

static flight_state_t ArmedHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_LAUNCH_DETECTED:      return STATE_FLIGHT_BURN;
        default: return STATE_FLIGHT_ARMED;
    }
}

static flight_state_t BurnHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_BURNOUT_DETECTED:     return STATE_FLIGHT_COAST;
        default: return STATE_FLIGHT_BURN;
    }
}

static flight_state_t CoastHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_APOGEE_DETECTED:      return STATE_FLIGHT_DESCEND_UNBRAKED;
        default: return STATE_FLIGHT_COAST;
    }
}

static flight_state_t UnbrakedDescendHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_DROGUE_DEPLOY:        return STATE_FLIGHT_DESCEND_DROGUE;
        default: return STATE_FLIGHT_DESCEND_UNBRAKED;
    }
}

static flight_state_t DrogueDescendHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_MAIN_DEPLOY:          return STATE_FLIGHT_DESCEND_MAIN;
        default: return STATE_FLIGHT_DESCEND_DROGUE;
    }
}

static flight_state_t MainDescendHandler(flight_event_t event) {
    switch (event) {
        case EVENT_FLIGHT_TOUCHDOWN:            return STATE_FLIGHT_LANDED;
        default:                                return STATE_FLIGHT_DESCEND_MAIN;
    }
}

static flight_state_t LandedHandler(flight_event_t event) {
    switch (event) {
        default: return STATE_FLIGHT_LANDED;
    }
}