#include <stdio.h>
#include "app.h"
#include "app_core.h"
#include "app_hardware.h"
#include "app_widgets.h"

#define APPACQ_OFFSET_SAMPLES 16U

static void APPACQ_Enter(void);
static void APPACQ_Process(void);
static void APPACQ_Render(void);
static void APPACQ_OnButton(Button_Event_t evt);

static bool APPACQ_ReadOffsetAverage(uint16_t *pd_a, uint16_t *pd_b);

const AppPageOps_t APP_PAGE_ACQUIRE_OPS = {
    .enter = APPACQ_Enter,
    .process = APPACQ_Process,
    .render = APPACQ_Render,
    .on_button = APPACQ_OnButton,
};

static void APPACQ_Enter(void)
{
    g_rt.acquire.step = ACQ_STEP_OFFSET_PREP;
    g_rt.acquire.pd_a_offset = 0U;
    g_rt.acquire.pd_b_offset = 0U;
    g_rt.acquire.offset_valid = false;
}

static void APPACQ_Process(void)
{
    uint16_t pd_a;
    uint16_t pd_b;

    switch (g_rt.acquire.step) {
        case ACQ_STEP_OFFSET_PREP:
            g_rt.acquire.step = ACQ_STEP_OFFSET_SAMPLE;
            APP_RequestRender();
            break;

        case ACQ_STEP_OFFSET_SAMPLE:
            if (!APPACQ_ReadOffsetAverage(&pd_a, &pd_b)) {
                APP_SetFault(APP_FAULT_ADC);
                return;
            }

            g_rt.acquire.pd_a_offset = pd_a;
            g_rt.acquire.pd_b_offset = pd_b;
            g_rt.acquire.offset_valid = true;
            g_rt.acquire.step = ACQ_STEP_OFFSET_DONE;
            APP_RequestRender();
            break;

        case ACQ_STEP_OFFSET_DONE:
        case ACQ_STEP_IDLE:
        default:
            break;
    }
}

static void APPACQ_Render(void)
{
    char line[32];

    switch (g_rt.acquire.step) {
        case ACQ_STEP_OFFSET_DONE:
            APPW_DrawFrame("Offset Zero");

            (void)snprintf(line, sizeof(line), "PD A: %4u", (unsigned)g_rt.acquire.pd_a_offset);
            APPW_WriteBodyLine(24, line);

            (void)snprintf(line, sizeof(line), "PD B: %4u", (unsigned)g_rt.acquire.pd_b_offset);
            APPW_WriteBodyLine(38, line);
            APPW_WriteBodyLine(56, "Short press resample");
            break;

        case ACQ_STEP_OFFSET_PREP:
        case ACQ_STEP_OFFSET_SAMPLE:
        case ACQ_STEP_IDLE:
        default:
            APPW_DrawFrame("Offset Zero");
            APPW_WriteBodyLine(28, "Sampling PD offsets");
            APPW_WriteBodyLine(44, "Please wait...");
            break;
    }
}

static void APPACQ_OnButton(Button_Event_t evt)
{
    if ((evt.id == BTN_ID_NONE) || (evt.action != BTN_ACT_SHORT)) {
        return;
    }

    if (g_rt.acquire.step != ACQ_STEP_OFFSET_DONE) {
        return;
    }

    g_rt.acquire.offset_valid = false;
    g_rt.acquire.step = ACQ_STEP_OFFSET_PREP;
    APP_RequestRender();
}

static bool APPACQ_ReadOffsetAverage(uint16_t *pd_a, uint16_t *pd_b)
{
    uint32_t sum_a;
    uint32_t sum_b;
    uint32_t i;
    PDADC_Frame_t frame;

    if ((pd_a == NULL) || (pd_b == NULL) || (g_hw->hpdadc == NULL)) {
        return false;
    }

    sum_a = 0U;
    sum_b = 0U;

    for (i = 0U; i < APPACQ_OFFSET_SAMPLES; ++i) {
        if (!PDADC_Read(g_hw->hpdadc, &frame)) {
            return false;
        }

        sum_a += frame.pd_a;
        sum_b += frame.pd_b;
    }

    *pd_a = (uint16_t)(sum_a / APPACQ_OFFSET_SAMPLES);
    *pd_b = (uint16_t)(sum_b / APPACQ_OFFSET_SAMPLES);
    return true;
}
