/*  System Supervisor
    Monitors system health, gets feedback and events from Tasks,
    Schedules and enables Error handling and recovery
    Provides comprehensive monitoring and on request feedback
*/

/*  How it works:
    Systems report status (that be simple info or errors) to the supervisor via a FreeRTOS event queue,
    To avoid event storms each type of event (combination component and event_id) only gets pushed to the queue once.
    A centralized record of all events in the queue is kept separate and updated with the event count.
    This record is accessed from a lookup table generated at compile time to avoid searching it for the right entry.

    A separate FreeRTOS thread contains the supervisor worker that gets the events from the queue, links them to their records
    and updates the status fields of their respective components
*/

#include "supervisor.h"
#include "supervisor_events.h"
#include "stm32h7xx_hal.h"
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define SUPERVISOR_COMPONENT_COUNT (sizeof(components) / sizeof(components[0]))
#define SUPERVISOR_RECORD_NONE UINT8_MAX

#ifndef SUPERVISOR_QUEUE_LENGTH
#define SUPERVISOR_QUEUE_LENGTH 32U
#endif

_Static_assert(SUPERVISOR_QUEUE_LENGTH > 0U &&
               SUPERVISOR_QUEUE_LENGTH <= UINT8_MAX,
               "Record indices must fit below the EMPTY sentinel");

typedef struct {
    uint32_t component_id;
    uint32_t event_offset;
    uint8_t  event_count;
    bool     info_enabled;
}SupervisorComponent_t;

typedef struct {
    uint32_t occurrence_count;
    uint32_t last_occurrence_ms;
} SupervisorEventRecord_t;

/* Component ID(only sequential), number of event types */
#define SUPERVISOR_COMPONENTS(X) \
    X(0, 10)/*Kernel*/           \
    X(SUP_COMPONENT_INTERBOARD, IBC_EVENT_COUNT)/*Inter Board Com*/  \
    X(2, 10)

/* Generate components[] */
#define COMPONENT_ENTRY(id, count) { .component_id = (id), .event_count = (count) },

static SupervisorComponent_t components[] = {
    SUPERVISOR_COMPONENTS(COMPONENT_ENTRY)
};
#undef COMPONENT_ENTRY

/* Generate a compile-time sum of event types for lookup table*/
#define COMPONENT_EVENT_COUNT(id, count) + (count)

enum {
    SUPERVISOR_TOTAL_EVENT_COUNT = 0 SUPERVISOR_COMPONENTS(COMPONENT_EVENT_COUNT)
};
#undef COMPONENT_EVENT_COUNT

/* Record lookup provides a static compile time generated table that holds the position of the events in the records table */
/* This limits us to a maxiumum of 255 simultaneously pending records*/
static uint8_t record_lookup[SUPERVISOR_TOTAL_EVENT_COUNT];

/* Event Record, same lenght as queue*/
static SupervisorEventRecord_t records[SUPERVISOR_QUEUE_LENGTH];
static uint8_t free_indices[SUPERVISOR_QUEUE_LENGTH];
static uint32_t free_count;

/* Private queue: only newly reserved records may be published to it. */
static QueueHandle_t SupervisorEventQueue;
static SupervisorDiagnostics_t diagnostics;

static void Supervisor_AddSaturating(uint32_t *value, uint32_t amount)
{
    *value = (amount > UINT32_MAX - *value) ? UINT32_MAX : *value + amount;
}

static void Supervisor_RecordInit(void)
{
    // Initialize lookup
    uint32_t offset = 0;

    for (size_t i = 0; i < SUPERVISOR_COMPONENT_COUNT; ++i) {
        components[i].event_offset = offset;
        offset += components[i].event_count;
    }

    memset(record_lookup, SUPERVISOR_RECORD_NONE,
           sizeof(record_lookup));

    // Initialize Record
    for (uint32_t i = 0; i < SUPERVISOR_QUEUE_LENGTH; ++i) {
        free_indices[i] = (uint8_t)i;
    }
    free_count = SUPERVISOR_QUEUE_LENGTH;
}

bool SV_Init(void)
{
    if (SupervisorEventQueue != NULL) {
        return true;
    }

    QueueHandle_t queue = xQueueCreate(SUPERVISOR_QUEUE_LENGTH, sizeof(SupervisorEventReport_t));
    if (queue == NULL) {
        return false;
    }
    Supervisor_RecordInit();
    SupervisorEventQueue = queue;

    /* Enable for the initial InterBoardCom tests. */
    (void)SV_SetInfoEnabled(SUP_COMPONENT_INTERBOARD, true);
    return true;
}

/**
 * @brief Report a new event to the supervisor. Do not call this function from an ISR context.
 * @param event Pointer to the event report structure
 * @return true if the event was reported successfully, false otherwise
 */
bool SV_ReportEvent(SupervisorEventReport_t *event)
{
    if (event == NULL || SupervisorEventQueue == NULL) {
        return false;
    }

    // Check if the component_id is valid
    if (event->component_id >= SUPERVISOR_COMPONENT_COUNT) {
        return false;
    }

    // Check if the event_id is valid for the given component
    const SupervisorComponent_t *component = &components[event->component_id];
    if (event->event_id >= component->event_count) {
        return false;
    }

    // Calculate the index in the record_lookup table
    uint32_t lookup_index = component->event_offset + event->event_id;

    // Lookup and record ownership changes share the same task critical section.
    taskENTER_CRITICAL();
    if (event->severity == SUP_SEVERITY_INFO && !component->info_enabled) {
        taskEXIT_CRITICAL();
        return true; /* Intentionally suppressed, not a reporting failure. */
    }
    uint8_t record_index = record_lookup[lookup_index];
    if (record_index != SUPERVISOR_RECORD_NONE) {
        Supervisor_AddSaturating(&records[record_index].occurrence_count, 1U);
        records[record_index].last_occurrence_ms = HAL_GetTick();
        taskEXIT_CRITICAL();
        return true;
    }

    if (free_count == 0U) {
        Supervisor_AddSaturating(&diagnostics.dropped_occurrences, 1U);
        taskEXIT_CRITICAL();
        return false;
    }

    record_index = free_indices[--free_count];
    uint32_t first_occurrence_ms = HAL_GetTick();
    records[record_index].occurrence_count = 1U;
    records[record_index].last_occurrence_ms = first_occurrence_ms;
    record_lookup[lookup_index] = record_index;
    taskEXIT_CRITICAL();

    event->first_occurrence_ms = first_occurrence_ms;
    if (xQueueSend(SupervisorEventQueue, event, 0) == pdTRUE) {
        return true;
    }

    /* Unexpected publication failure: account for all accumulated occurrences
     * and restore ownership. There is no queued item that could consume it. */
    taskENTER_CRITICAL();
    Supervisor_AddSaturating(&diagnostics.queue_send_failures, 1U);
    Supervisor_AddSaturating(&diagnostics.dropped_occurrences, records[record_index].occurrence_count);
    record_lookup[lookup_index] = SUPERVISOR_RECORD_NONE;
    free_indices[free_count++] = record_index;
    taskEXIT_CRITICAL();
    return false;
}

/**
 * @brief Report a new event to the supervisor. ISR safe version.
 * @param event Pointer to the event report structure
 * @param higher_priority_task_woken Pointer to a variable that will be set to pdTRUE if a higher priority task is woken by the function
 * @return true if the event was reported successfully, false otherwise
 */
bool SV_ReportEventFromISR(SupervisorEventReport_t *event, BaseType_t *higher_priority_task_woken)
{
    if (event == NULL || SupervisorEventQueue == NULL) {
        return false;
    }

    // Check if the component_id is valid
    if (event->component_id >= SUPERVISOR_COMPONENT_COUNT) {
        return false;
    }

    // Check if the event_id is valid for the given component
    const SupervisorComponent_t *component = &components[event->component_id];
    if (event->event_id >= component->event_count) {
        return false;
    }

    // Calculate the index in the record_lookup table
    uint32_t lookup_index = component->event_offset + event->event_id;

    // Share the same interrupt mask protection as task-side record updates.
    UBaseType_t saved_mask = taskENTER_CRITICAL_FROM_ISR();
    if (event->severity == SUP_SEVERITY_INFO && !component->info_enabled) {
        taskEXIT_CRITICAL_FROM_ISR(saved_mask);
        return true; /* Intentionally suppressed, not a reporting failure. */
    }
    uint8_t record_index = record_lookup[lookup_index];
    if (record_index != SUPERVISOR_RECORD_NONE) {
        Supervisor_AddSaturating(&records[record_index].occurrence_count, 1U);
        records[record_index].last_occurrence_ms = HAL_GetTick();
        taskEXIT_CRITICAL_FROM_ISR(saved_mask);
        return true;
    }

    if (free_count == 0U) {
        Supervisor_AddSaturating(&diagnostics.dropped_occurrences, 1U);
        taskEXIT_CRITICAL_FROM_ISR(saved_mask);
        return false;
    }

    record_index = free_indices[--free_count];
    uint32_t first_occurrence_ms = HAL_GetTick();
    records[record_index].occurrence_count = 1U;
    records[record_index].last_occurrence_ms = first_occurrence_ms;
    record_lookup[lookup_index] = record_index;
    taskEXIT_CRITICAL_FROM_ISR(saved_mask);

    event->first_occurrence_ms = first_occurrence_ms;
    if (xQueueSendFromISR(SupervisorEventQueue, event, higher_priority_task_woken) == pdTRUE) {
        return true;
    }

    /* Unexpected publication failure: account for all accumulated occurrences
     * and restore ownership. There is no queued item that could consume it. */
    saved_mask = taskENTER_CRITICAL_FROM_ISR();
    Supervisor_AddSaturating(&diagnostics.queue_send_failures, 1U);
    Supervisor_AddSaturating(&diagnostics.dropped_occurrences, records[record_index].occurrence_count);
    record_lookup[lookup_index] = SUPERVISOR_RECORD_NONE;
    free_indices[free_count++] = record_index;
    taskEXIT_CRITICAL_FROM_ISR(saved_mask);
    return false;
}

/**
 * @brief Process events in the supervisor queue.
 */
void SV_ProcessEvents(void)
{
    if (SupervisorEventQueue == NULL) {
        return;
    }

    SupervisorEventReport_t queued;
    for (uint32_t processed = 0; processed < SUPERVISOR_QUEUE_LENGTH; ++processed) {
        if (xQueueReceive(SupervisorEventQueue, &queued, 0) != pdTRUE) {
            break;
        }

        /* The private queue contains only reports validated by ReportEvent. */
        uint32_t lookup_index = components[queued.component_id].event_offset + queued.event_id;
        SupervisorEventSnapshot_t snapshot = {
            .event = queued
        };

        taskENTER_CRITICAL();
        uint8_t record_index = record_lookup[lookup_index];
        if (record_index >= SUPERVISOR_QUEUE_LENGTH ||
            free_count >= SUPERVISOR_QUEUE_LENGTH) {
            Supervisor_AddSaturating(&diagnostics.record_errors, 1U);
            taskEXIT_CRITICAL();
            continue;
        }

        snapshot.occurrence_count = records[record_index].occurrence_count;
        snapshot.last_occurrence_ms = records[record_index].last_occurrence_ms;
        record_lookup[lookup_index] = SUPERVISOR_RECORD_NONE;
        free_indices[free_count++] = record_index;
        taskEXIT_CRITICAL();

        // Handle event, log, it, update component status, etc.
    }
}

/**
 * @brief Create a new event report.
 * @param component_id The ID of the component that generated the event, check supervisor.c to validate.
 * @param event_id The ID of the event.
 * @param severity The severity of the event.
 * @param argument An argument for the event.
 * @return A new event report.
 */
SupervisorEventReport_t SV_EventReport(uint32_t component_id, uint32_t event_id, SupervisorSeverity_t severity, uint32_t argument)
{
    SupervisorEventReport_t report = {
        .component_id = component_id,
        .event_id = event_id,
        .severity = severity,
        .argument = argument,
        .metric_valid_mask = 0U
    };
    return report;
}

bool SV_SetInfoEnabled(uint32_t component_id, bool enabled)
{
    if (component_id >= SUPERVISOR_COMPONENT_COUNT) {
        return false;
    }

    taskENTER_CRITICAL();
    components[component_id].info_enabled = enabled;
    taskEXIT_CRITICAL();

    return true;
}

/**
 * @brief Get the current diagnostics.
 * @param snapshot A pointer to the snapshot to fill.
 */
void SV_GetDiagnostics(SupervisorDiagnostics_t *snapshot)
{
    if (snapshot != NULL) {
        taskENTER_CRITICAL();
        *snapshot = diagnostics;
        taskEXIT_CRITICAL();
    }
}
