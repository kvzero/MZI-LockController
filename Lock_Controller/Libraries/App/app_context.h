#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include "button.h"
#include "hv_amp.h"
#include "hv_dac.h"
#include "pd_adc.h"
#include "st7735.h"

typedef enum {
    SYS_STATE_INIT = 0,
    SYS_STATE_CALIBRATING,
    SYS_STATE_SCANNING,
    SYS_STATE_LOCKED,
    SYS_STATE_FAULT,
} SystemState_t;

typedef struct {
    Button_Handle_t *hbtn;
    HV_DAC_Handle_t *hvdac;
    HVAMP_Handle_t  *hvamp;
    ST7735_Handle_t *hlcd;
    PDADC_Handle_t  *hpdadc;
} App_Context_t;

extern App_Context_t * const g_app;

#endif /* APP_CONTEXT_H */
