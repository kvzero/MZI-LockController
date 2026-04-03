#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Top-level pages. Each page owns its own process, render, and button handlers.
 */
typedef enum {
    APP_PAGE_WAIT = 0,
    APP_PAGE_ACQUIRE,
    APP_PAGE_FAULT,
    APP_PAGE_COUNT,
} AppPage_t;

typedef enum {
    APP_FAULT_NONE = 0,
    APP_FAULT_HVAMP,
    APP_FAULT_ADC,
} FaultCode_t;

/**
 * @brief Acquire-page internal steps.
 *
 * Only the acquire page uses this state machine. Top-level page switching is handled by AppPage_t.
 */
typedef enum {
    ACQ_STEP_IDLE = 0,
    ACQ_STEP_OFFSET_PREP,
    ACQ_STEP_OFFSET_SAMPLE,
    ACQ_STEP_OFFSET_DONE,
} AcquireStep_t;

typedef struct {
    AcquireStep_t step;
    uint16_t      pd_a_offset;
    uint16_t      pd_b_offset;
    bool          offset_valid;
} AppAcquireRuntime_t;

/**
 * @brief Front-end runtime shared by all app pages.
 */
typedef struct {
    AppPage_t          page;
    FaultCode_t        fault;
    bool               ui_dirty;
    AppAcquireRuntime_t acquire;
} AppRuntime_t;

extern AppRuntime_t g_rt;

#endif /* APP_CONTEXT_H */
