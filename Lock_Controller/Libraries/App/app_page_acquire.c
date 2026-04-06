#include <stdio.h>
#include "app.h"
#include "app_core.h"
#include "app_hardware.h"
#include "app_rtloop.h"
#include "app_scan.h"
#include "app_widgets.h"

#define APPACQ_OFFSET_SAMPLES 16U
#define APPACQ_SCAN_CYCLES    3U

static void APPACQ_Enter(void);
static void APPACQ_Process(void);
static void APPACQ_Render(void);
static void APPACQ_OnButton(Button_Event_t evt);

static bool APPACQ_ReadOffsetAverage(uint16_t *iout, uint16_t *iref);

const AppPageOps_t APP_PAGE_ACQUIRE_OPS = {
    .enter = APPACQ_Enter,
    .process = APPACQ_Process,
    .render = APPACQ_Render,
    .on_button = APPACQ_OnButton,
};

static void APPACQ_Enter(void)
{
    APPSCAN_Reset();
    g_rt.acquire.step = ACQ_STEP_OFFSET_PREP;
    g_rt.acquire.offset.iout_offset_raw = 0U;
    g_rt.acquire.offset.iref_offset_raw = 0U;
    g_rt.acquire.offset.valid = false;
    g_rt.acquire.scan.cycles_done = 0U;
    g_rt.acquire.scan.cycles_total = 0U;
    g_rt.acquire.scan.contrast_q15 = 0U;
    g_rt.acquire.scan.r_max_q15 = 0U;
    g_rt.acquire.scan.r_min_q15 = 0U;
    g_rt.acquire.scan.r_target_q15 = 0U;
    g_rt.acquire.scan.valid = false;
}

static void APPACQ_Process(void)
{
    const AppScanResult_t *scan_result;
    uint16_t iout;
    uint16_t iref;

    switch (g_rt.acquire.step) {
        case ACQ_STEP_OFFSET_PREP:
            g_rt.acquire.step = ACQ_STEP_OFFSET_RUN;
            APP_RequestRender();
            break;

        case ACQ_STEP_OFFSET_RUN:
            if (!APPACQ_ReadOffsetAverage(&iout, &iref)) {
                APP_SetFault(APP_FAULT_ADC);
                return;
            }

            g_rt.acquire.offset.iout_offset_raw = iout;
            g_rt.acquire.offset.iref_offset_raw = iref;
            g_rt.acquire.offset.valid = true;
            g_rt.acquire.step = ACQ_STEP_RESULT_READY;
            APP_RequestRender();
            break;

        case ACQ_STEP_SCAN_PREP:
            g_rt.acquire.scan.cycles_done = 0U;
            g_rt.acquire.scan.cycles_total = APPACQ_SCAN_CYCLES;
            g_rt.acquire.scan.r_max_q15 = 0U;
            g_rt.acquire.scan.r_min_q15 = 0U;
            g_rt.acquire.scan.r_target_q15 = 0U;
            g_rt.acquire.scan.valid = false;

            if (!APPSCAN_Start(APPACQ_SCAN_CYCLES,
                               g_rt.acquire.offset.iout_offset_raw,
                               g_rt.acquire.offset.iref_offset_raw)) {
                APP_SetFault(APP_FAULT_ADC);
                return;
            }

            g_rt.acquire.step = ACQ_STEP_SCAN_RUN;
            APP_RequestRender();
            break;

        case ACQ_STEP_SCAN_RUN:
            if (APPRTLOOP_HasError()) {
                APP_SetFault(APP_FAULT_ADC);
                return;
            }

            scan_result = APPSCAN_GetResult();
            if (!APPSCAN_IsDone() || (scan_result == NULL) || !scan_result->valid) {
                break;
            }

            g_rt.acquire.scan = *scan_result;
            g_rt.acquire.step = ACQ_STEP_RESULT_READY;
            APP_RequestRender();
            break;

        case ACQ_STEP_RESULT_READY:
        case ACQ_STEP_IDLE:
        default:
            break;
    }
}

static void APPACQ_Render(void)
{
    char line[32];
    uint32_t contrast_permille;

    switch (g_rt.acquire.step) {
        case ACQ_STEP_RESULT_READY:
            if (!g_rt.acquire.scan.valid) {
                APPW_DrawFrame("Offset Zero");

                (void)snprintf(line, sizeof(line), "Iout: %4u", (unsigned)g_rt.acquire.offset.iout_offset_raw);
                APPW_WriteBodyLine(22, line);

                (void)snprintf(line, sizeof(line), "Iref: %4u", (unsigned)g_rt.acquire.offset.iref_offset_raw);
                APPW_WriteBodyLine(36, line);
                APPW_WriteBodyLine(54, "Turn laser on");
                APPW_WriteBodyLine(68, "Short press scan");
                break;
            }

            APPW_DrawFrame("Scan Result");
            contrast_permille = ((uint32_t)g_rt.acquire.scan.contrast_q15 * 1000U + 16384U) >> 15;

            (void)snprintf(line, sizeof(line), "Ctr: %3u.%1u%%",
                           (unsigned)(contrast_permille / 10U),
                           (unsigned)(contrast_permille % 10U));
            APPW_WriteBodyLine(18, line);

            (void)snprintf(line, sizeof(line), "Cycles: %u/%u",
                           (unsigned)g_rt.acquire.scan.cycles_done,
                           (unsigned)g_rt.acquire.scan.cycles_total);
            APPW_WriteBodyLine(32, line);

            (void)snprintf(line, sizeof(line), "Rmax: %5u", (unsigned)g_rt.acquire.scan.r_max_q15);
            APPW_WriteBodyLine(46, line);

            (void)snprintf(line, sizeof(line), "Rmin: %5u", (unsigned)g_rt.acquire.scan.r_min_q15);
            APPW_WriteBodyLine(60, line);

            (void)snprintf(line, sizeof(line), "Rtgt: %5u", (unsigned)g_rt.acquire.scan.r_target_q15);
            APPW_WriteBodyLine(74, line);
            break;

        case ACQ_STEP_OFFSET_PREP:
        case ACQ_STEP_OFFSET_RUN:
        case ACQ_STEP_IDLE:
        default:
            APPW_DrawFrame("Offset Zero");
            APPW_WriteBodyLine(28, "Sampling PD offsets");
            APPW_WriteBodyLine(44, "Please wait...");
            break;

        case ACQ_STEP_SCAN_PREP:
        case ACQ_STEP_SCAN_RUN:
            APPW_DrawFrame("Fringe Scan");

            (void)snprintf(line, sizeof(line), "Cycles: %u/%u",
                           (unsigned)g_rt.acquire.scan.cycles_done,
                           (unsigned)g_rt.acquire.scan.cycles_total);
            APPW_WriteBodyLine(24, line);
            APPW_WriteBodyLine(42, "Laser on, scanning");
            APPW_WriteBodyLine(56, "Please wait...");
            break;
    }
}

static void APPACQ_OnButton(Button_Event_t evt)
{
    if ((evt.id == BTN_ID_NONE) || (evt.action != BTN_ACT_SHORT)) {
        return;
    }

    if (g_rt.acquire.step != ACQ_STEP_RESULT_READY) {
        return;
    }

    if (!g_rt.acquire.scan.valid) {
        g_rt.acquire.step = ACQ_STEP_SCAN_PREP;
        APP_RequestRender();
    }
}

static bool APPACQ_ReadOffsetAverage(uint16_t *iout, uint16_t *iref)
{
    uint32_t sum_a;
    uint32_t sum_b;
    uint32_t i;
    PDADC_Frame_t frame;

    if ((iout == NULL) || (iref == NULL) || (g_hw->hpdadc == NULL)) {
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

    *iout = (uint16_t)(sum_a / APPACQ_OFFSET_SAMPLES);
    *iref = (uint16_t)(sum_b / APPACQ_OFFSET_SAMPLES);
    return true;
}
