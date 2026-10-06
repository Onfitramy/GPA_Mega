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
    uint32_t argument;
    uint32_t metric_valid_mask;
    uint32_t metrics[SUPERVISOR_METRIC_COUNT];
} SupervisorEventReport_t;

typedef struct {
    SupervisorEventReport_t event;
    uint32_t first_occurrence_ms;
    uint32_t occurrence_count;
    uint32_t last_occurrence_ms;
} SupervisorEventSnapshot_t;

typedef struct {
    uint32_t dropped_occurrences;
    uint32_t queue_send_failures;
    uint32_t record_errors;
} SupervisorDiagnostics_t;

/* Call from initialization before any reporting/consumer tasks start.
 * Repeated successful initialization is a no-op; allocation failure is retryable. */
bool Supervisor_Init(void);

/* Task context only; callable by multiple tasks. Never waits for queue space.
 * The caller must keep the report unchanged for the duration of this call. */
bool Supervisor_ReportEvent(const SupervisorEventReport_t *event);

/* Task context only; exactly one task owns consumption. Each call processes
 * at most the queue length, then returns so other supervision work can run. */
void Supervisor_ProcessEvents(void);

void Supervisor_GetDiagnostics(SupervisorDiagnostics_t *snapshot);

/* Override the weak default to evaluate events/recovery in the consumer task.
 * The snapshot is valid only during the call; copy it if retaining it. */
void Supervisor_HandleEvent(const SupervisorEventSnapshot_t *snapshot);

#endif
