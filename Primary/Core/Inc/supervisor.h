#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>

#define SUPERVISOR_METRIC_COUNT 4U

typedef enum {
    SUP_SEVERITY_NONE = 0,
    SUP_SEVERITY_INFO,
    SUP_SEVERITY_WARNING,
    SUP_SEVERITY_ERROR,
    SUP_SEVERITY_FATAL
} SupervisorSeverity_t;

typedef struct {
    uint32_t component_id;
    uint32_t event_id;
    SupervisorSeverity_t severity;
    uint32_t first_occurrence_ms;
    uint32_t argument;
    uint32_t metric_valid_mask;
    uint32_t metrics[SUPERVISOR_METRIC_COUNT];
} SupervisorEventReport_t;

typedef struct {
    SupervisorEventReport_t event;
    uint32_t occurrence_count;
    uint32_t last_occurrence_ms;
} SupervisorEventSnapshot_t;

typedef struct {
    uint32_t dropped_occurrences;
    uint32_t queue_send_failures;
    uint32_t record_errors;
} SupervisorDiagnostics_t;

//For readability all public functions are prefixed with SV_

/* Call from initialization before any reporting/consumer tasks start.
 * Repeated successful initialization is a no-op; allocation failure is retryable. */
bool SV_Init(void);

/* Task context only; callable by multiple tasks. Never waits for queue space.
 * The caller must keep the report unchanged for the duration of this call. */
bool SV_ReportEvent(SupervisorEventReport_t *event);

/* Task context only; exactly one task owns consumption. Each call processes
 * at most the queue length, then returns so other supervision work can run. */
void SV_ProcessEvents(void);

/**
 * @brief Create a new event report.
 * @param component_id The ID of the component that generated the event, check supervisor.c to validate.
 * @param event_id The ID of the event.
 * @param severity The severity of the event.
 * @param argument An argument for the event.
 * @return A new event report.
 */
SupervisorEventReport_t SV_EventReport(uint32_t component_id, uint32_t event_id, SupervisorSeverity_t severity, uint32_t argument);

void SV_GetDiagnostics(SupervisorDiagnostics_t *snapshot);

#endif
