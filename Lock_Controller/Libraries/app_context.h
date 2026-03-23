/*
 * app_context.h
 */

#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include "hv_dac.h"
#include "hv_amp.h"
#include "button.h"
#include "st7735.h"

/**
 * @brief  System Operational States.
 */
typedef enum {
    SYS_STATE_INIT = 0,
    SYS_STATE_CALIBRATING,
    SYS_STATE_SCANNING,
    SYS_STATE_LOCKED,       /* PID closed-loop active */
    SYS_STATE_FAULT         /* HV Amp fault or system error */
} SystemState_t;

/**
 * @brief   Holds pointers to hardware handles and global telemetry.
 */
typedef struct {
    /* Hardware Handle Pointers (The Registry) */
    Button_Handle_t *hbtn;
    HV_DAC_Handle_t *hvdac;
    HVAMP_Handle_t  *hvamp;
    ST7735_Handle_t *hlcd;

} App_Context_t;

/**
 * @brief  Global singleton pointer to the app context.
 *         Accessible from any module.
 */
extern App_Context_t * const g_app;
extern SystemState_t * const g_state;

#endif /* APP_CONTEXT_H */
