#ifndef APP_HW_H
#define APP_HW_H

#include "button.h"
#include "hv_amp.h"
#include "hv_dac.h"
#include "pd_adc.h"
#include "st7735.h"

/**
 * @brief Hardware registry owned by main.c.
 */
typedef struct {
    Button_Handle_t *hbtn;
    HV_DAC_Handle_t *hvdac;
    HVAMP_Handle_t  *hvamp;
    ST7735_Handle_t *hlcd;
    PDADC_Handle_t  *hpdadc;
} AppHardware_t;

extern AppHardware_t * const g_hw;

#endif /* APP_HW_H */
