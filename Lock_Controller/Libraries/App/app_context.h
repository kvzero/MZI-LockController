#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include <stdbool.h>

/**
 * @brief Top-level pages. Each page owns its own process, render, and button handlers.
 */
typedef enum {
    APP_PAGE_WAIT = 0,
    APP_PAGE_ACQUIRE,
    APP_PAGE_LOCK,
    APP_PAGE_FAULT,
    APP_PAGE_COUNT,
} AppPage_t;

typedef enum {
    APP_FAULT_NONE = 0,
    APP_FAULT_HVAMP,
    APP_FAULT_ADC,
    APP_FAULT_LOCK,
    APP_FAULT_CONTRAST,
} FaultCode_t;

/**
 * @brief Front-end runtime shared by all app pages.
 */
typedef struct {
    AppPage_t   page;
    FaultCode_t fault;
    bool        ui_dirty;
} AppRuntime_t;

extern AppRuntime_t g_rt;

#endif /* APP_CONTEXT_H */
