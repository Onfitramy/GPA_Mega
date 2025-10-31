#include "statemachine.h"

#include "Buzzer.h"
#include "W25Q1.h"
#include "xBee.h"

StateMachine_t pu_sm;

uint8_t status_data = 0;
uint8_t selftest_tries = 0;

/* --- Event handlers --- */

static sm_state_t FaultHandler(sm_event_t event) {
    switch (event) {
        case EVENT_FAULT_CLEARED:       return STATE_OPERATIONAL;
        default: return STATE_FAULT;
    }
}

static sm_state_t StartupHandler(sm_event_t event) {
    switch (event) {
        case EVENT_STARTUP_COMPLETE:    return STATE_INIT;
        default: return STATE_STARTUP;
    }
}

static sm_state_t InitHandler(sm_event_t event) {
    switch (event) {
        case EVENT_INITIALIZED:         return STATE_OPERATIONAL;
        default: return STATE_INIT;
    }
}

static sm_state_t OperationalHandler(sm_event_t event) {
    switch (event) {
        case EVENT_FAULT:               return STATE_FAULT;
        case EVENT_BATTERY_LOW:         return STATE_BATTERY_LOW;
        default: return STATE_OPERATIONAL;
    }
}

static sm_state_t BatteryLowHandler(sm_event_t event) {
    switch (event) {
        case EVENT_FAULT:               return STATE_FAULT;
        case EVENT_BATTERY_OK:          return STATE_OPERATIONAL;
        case EVENT_BATTERY_CRITICAL:    return STATE_BATTERY_CRITICAL;
        default: return STATE_BATTERY_LOW;
    }
}

static sm_state_t BatteryCriticalHandler(sm_event_t event) {
    switch (event) {
        case EVENT_FAULT:               return STATE_FAULT;
        case EVENT_BATTERY_LOW:         return STATE_BATTERY_LOW;
        default: return STATE_BATTERY_CRITICAL;
    }
}

/* --- Entry actions --- */
static void FaultEntry(StateMachine_t *sm) {
    status_data = 0;
}
static void StartupEntry(StateMachine_t *sm) {
    uint32_t flashid = W25Q1_ReadID(); //Check if the FLASH works, flashid = 0xEF4017
    buzzerInit();
    XBee_Init();
}
static void InitEntry(StateMachine_t *sm) {}
static void OperationalEntry(StateMachine_t *sm) {}
static void BatteryLowEntry(StateMachine_t *sm) {}
static void BatteryCriticalEntry(StateMachine_t *sm) {}

/* --- Do actions --- */
static void FaultDo(StateMachine_t *sm, uint16_t freq) {
    // Selftest is called with 10 Hz
    if (freq != 10) return;

    if ((status_data & 0x01) == 0x01) {
        // all selftests passed
        StateMachine_Dispatch(sm, EVENT_FAULT_CLEARED);
        status_data |= (1 << 7);
    } else {
        selftest_tries++;
        status_data |= INA219_SelfTest();
    }
}
static void StartupDo(StateMachine_t *sm, uint16_t freq) {
    StateMachine_Dispatch(&pu_sm, EVENT_STARTUP_COMPLETE);
}
static void InitDo(StateMachine_t *sm, uint16_t freq) {
    // Selftest is called with 10 Hz
    if (freq != 10) return;

    if ((status_data & 0x01) == 0x01) {
        // all selftests passed
        StateMachine_Dispatch(sm, EVENT_INITIALIZED);
        status_data |= (1 << 7);
    } else {
        selftest_tries++;
        status_data |= INA219_SelfTest();
    }
}
static void OperationalDo(StateMachine_t *sm, uint16_t freq) {
    if (freq != 10) return;

    if (INA219_readAll(&health) != HAL_OK) StateMachine_Dispatch(&pu_sm, EVENT_FAULT);
    if (health.voltage.bus_pu_bat < 6.6) StateMachine_Dispatch(&pu_sm, EVENT_BATTERY_LOW);
}
static void BatteryLowDo(StateMachine_t *sm, uint16_t freq) {
    if (freq != 10) return;

    if (INA219_readAll(&health) != HAL_OK) StateMachine_Dispatch(&pu_sm, EVENT_FAULT);
    if (health.voltage.bus_pu_bat < 6.0) StateMachine_Dispatch(&pu_sm, EVENT_BATTERY_CRITICAL);
    if (health.voltage.bus_pu_bat > 6.8) StateMachine_Dispatch(&pu_sm, EVENT_BATTERY_OK);
}
static void BatteryCriticalDo(StateMachine_t *sm, uint16_t freq) {
    if (freq != 10) return;

    if (INA219_readAll(&health) != HAL_OK) StateMachine_Dispatch(&pu_sm, EVENT_FAULT);
    if (health.voltage.bus_pu_bat > 6.2) StateMachine_Dispatch(&pu_sm, EVENT_BATTERY_LOW);
}

/* --- Exit actions --- */
static void FaultExit(StateMachine_t *sm) {}
static void StartupExit(StateMachine_t *sm) {}
static void InitExit(StateMachine_t *sm) {}
static void OperationalExit(StateMachine_t *sm) {}
static void BatteryLowExit(StateMachine_t *sm) {}
static void BatteryCriticalExit(StateMachine_t *sm) {}

/* --- Lookup tables for state functions --- */
static StateHandler_t stateHandlerTable[STATE_MAX] = {
    FaultHandler,
    StartupHandler,
    InitHandler,
    OperationalHandler,
    BatteryLowHandler,
    BatteryCriticalHandler
};

static StateEntry_t stateEntryTable[STATE_MAX] = {
    FaultEntry,
    StartupEntry,
    InitEntry,
    OperationalEntry,
    BatteryLowEntry,
    BatteryCriticalEntry
};

static StateDo_t stateDoTable[STATE_MAX] = {
    FaultDo,
    StartupDo,
    InitDo,
    OperationalDo,
    BatteryLowDo,
    BatteryCriticalDo
};

static StateExit_t stateExitTable[STATE_MAX] = {
    FaultExit,
    StartupExit,
    InitExit,
    OperationalExit,
    BatteryLowExit,
    BatteryCriticalExit
};

/* --- Lookup tables for timing constraints --- */
uint32_t minEventDelayTable[EVENT_MAX] = {
    0,
    0,
    0,
    0,
    0,
    0,
    0
};

uint32_t maxEventDelayTable[STATE_MAX] = {
    0,
    0,
    0,
    0,
    0,
    0
};

/* --- Action handler functions --- */
static void StateEntryHandler(StateMachine_t *sm, sm_state_t state) {
    // set entry timestamp
    sm->timestamp_us = uwTick;

    // handle entry for each state
    stateEntryTable[state](sm);
}

static void StateExitHandler(StateMachine_t *sm, sm_state_t state) {
    stateExitTable[state](sm);
}

static void StateDoHandler(StateMachine_t *sm, sm_state_t state, uint16_t freq) {
    stateDoTable[state](sm, freq);
}

/* --- User functions --- */
void StateMachine_Init(StateMachine_t *sm, sm_state_t initialState) {
    sm->currentState = initialState;

    // call entry action for initial state
    StateEntryHandler(sm, sm->currentState);
}

void StateMachine_Dispatch(StateMachine_t *sm, sm_event_t event) {
    if (sm->currentState >= STATE_MAX) return;

    // TODO: store event on Flash & SD

    // store old flight state
    sm_state_t oldState = sm->currentState;

    // handle event and retrieve new flight state
    sm_state_t newState = stateHandlerTable[oldState](event);

    // prevent exit and entry actions if no state change
    if (newState == oldState || newState >= STATE_MAX) return;

    // don't update state if minimum entry time delay for event hasn't elapsed yet
    if (minEventDelayTable[event] > (uwTick - sm->timestamp_us)) return;

    // exit old state
    StateExitHandler(sm, oldState);

    // update flight state
    sm->currentState = newState;

    // stop timer
    HAL_TIM_Base_Stop_IT(&htim7);

    // start timer to enforce maximum delay until event
    if ((tim7_target_ms = maxEventDelayTable[newState])) {
        HAL_TIM_Base_Start_IT(&htim7);
    }

    // enter new state
    StateEntryHandler(sm, newState);
}

void StateMachine_ForceState(StateMachine_t *sm, sm_state_t newState) {
    if (sm->currentState >= STATE_MAX) return;

    // TODO: store command on Flash & SD

    // store old flight state
    sm_state_t oldState = sm->currentState;

    // prevent exit and entry actions if no state change
    if (newState == oldState || newState >= STATE_MAX) return;

    // exit old state
    StateExitHandler(sm, oldState);

    // update flight state
    sm->currentState = newState;

    // stop timer
    HAL_TIM_Base_Stop_IT(&htim7);

    // start timer to enforce maximum delay until event
    if ((tim7_target_ms = maxEventDelayTable[newState])) {
        HAL_TIM_Base_Start_IT(&htim7);
    }

    // enter new state
    StateEntryHandler(sm, newState);
}

void StateMachine_DoActions(StateMachine_t *sm, uint16_t freq) {
    // perform standard state actions
    StateDoHandler(sm, sm->currentState, freq);
}