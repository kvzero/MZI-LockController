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
    APP_FAULT_LOCK,
} FaultCode_t;

/**
 * @brief Acquire-page internal steps.
 *
 * Only the acquire page uses this state machine. Top-level page switching is handled by AppPage_t.
 */
typedef enum {
    ACQ_STEP_IDLE = 0,
    ACQ_STEP_OFFSET_PREP,
    ACQ_STEP_OFFSET_RUN,
    ACQ_STEP_SCAN_PREP,
    ACQ_STEP_SCAN_RUN,
    ACQ_STEP_SOFTLOCK_PREP,
    ACQ_STEP_SOFTLOCK_RUN,
    ACQ_STEP_RESULT_READY,
} AcquireStep_t;

typedef struct {
    uint16_t iout_offset_raw;
    uint16_t iref_offset_raw;
    bool     valid;
} AppOffsetResult_t;

typedef struct {
    uint8_t  cycles_done;
    uint8_t  cycles_total;
    uint16_t contrast_q15;
    uint32_t r_max_q15;
    uint32_t r_min_q15;
    uint32_t r_target_q15;
    bool     valid;
} AppScanResult_t;

typedef struct {
    AcquireStep_t     step;
    AppOffsetResult_t offset;
    AppScanResult_t   scan;
} AppAcquireRuntime_t;

typedef struct {
    bool     active;
    bool     soft_locked;
    bool     error;
    int8_t   polarity;
    uint32_t r_target_q15;
    uint32_t r_now_q15;
    int32_t  error_q15;
    uint16_t capture_raw;
    uint16_t output_raw;
} AppLockRuntime_t;

/**
 * @brief Front-end runtime shared by all app pages.
 */
typedef struct {
    AppPage_t          page;
    FaultCode_t        fault;
    bool               ui_dirty;
    AppAcquireRuntime_t acquire;
    AppLockRuntime_t    lock;
} AppRuntime_t;

extern AppRuntime_t g_rt;

#endif /* APP_CONTEXT_H */
