#include <stdio.h>
#include "stm32g4xx_hal.h"
#include "app.h"
#include "app_core.h"
#include "app_hardware.h"
#include "app_lock.h"
#include "app_rtloop.h"
#include "app_scan.h"
#include "app_widgets.h"

#define APPACQ_OFFSET_SAMPLES 16U
#define APPACQ_SCAN_CYCLES    3U
#define APPACQ_SUMMARY_HOLD_MS 1000U
#define APPACQ_CONTRAST_MIN_Q15 16384U

static void APPACQ_Enter(void);
static void APPACQ_Process(void);
static void APPACQ_Render(void);
static void APPACQ_OnButton(Button_Event_t evt);

static void APPACQ_GotoStep(AcquireStep_t step);
static bool APPACQ_StepElapsed(uint32_t elapsed_ms);
static bool APPACQ_ReadOffsetAverage(uint16_t *iout, uint16_t *iref);

static uint32_t s_step_enter_ms;

const AppPageOps_t APP_PAGE_ACQUIRE_OPS = {
    .enter = APPACQ_Enter,
    .process = APPACQ_Process,
    .render = APPACQ_Render,
    .on_button = APPACQ_OnButton,
};

static void APPACQ_Enter(void)
{
    APPLOCK_Reset();
    APPSCAN_Reset();
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
    g_rt.lock.active = false;
    g_rt.lock.soft_locked = false;
    g_rt.lock.hard_locked = false;
    g_rt.lock.resonance_done = false;
    g_rt.lock.error = false;
    g_rt.lock.polarity = 0;
    g_rt.lock.r_target_q15 = 0U;
    g_rt.lock.r_now_q15 = 0U;
    g_rt.lock.error_q15 = 0;
    g_rt.lock.capture_raw = 0U;
    g_rt.lock.output_raw = 0U;
    g_rt.lock.resonance_freq_hz = 0UL;
    g_rt.lock.fn_hz = 0UL;
    APPACQ_GotoStep(ACQ_STEP_OFFSET_PREP);
}

static void APPACQ_Process(void)
{
    const AppLockRuntime_t *lock_result;
    const AppScanResult_t *scan_result;
    uint16_t iout;
    uint16_t iref;

    switch (g_rt.acquire.step) {
        case ACQ_STEP_OFFSET_PREP:
            APPACQ_GotoStep(ACQ_STEP_OFFSET_RUN);
            break;

        case ACQ_STEP_OFFSET_RUN:
            if (!APPACQ_ReadOffsetAverage(&iout, &iref)) {
                APP_SetFault(APP_FAULT_ADC);
                return;
            }

            g_rt.acquire.offset.iout_offset_raw = iout;
            g_rt.acquire.offset.iref_offset_raw = iref;
            g_rt.acquire.offset.valid = true;
            APPACQ_GotoStep(ACQ_STEP_OFFSET_DONE);
            break;

        case ACQ_STEP_SCAN_PREP:
            g_rt.acquire.scan.cycles_done = 0U;
            g_rt.acquire.scan.cycles_total = APPACQ_SCAN_CYCLES;
            g_rt.acquire.scan.contrast_q15 = 0U;
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

            APPACQ_GotoStep(ACQ_STEP_SCAN_RUN);
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
            APPACQ_GotoStep(ACQ_STEP_SCAN_DONE);
            break;

        case ACQ_STEP_SCAN_DONE:
            if (APPACQ_StepElapsed(APPACQ_SUMMARY_HOLD_MS)) {
                APPACQ_GotoStep(ACQ_STEP_SOFTLOCK_PREP);
            }
            break;

        case ACQ_STEP_SOFTLOCK_PREP:
            g_rt.lock.active = false;
            g_rt.lock.soft_locked = false;
            g_rt.lock.hard_locked = false;
            g_rt.lock.resonance_done = false;
            g_rt.lock.error = false;
            g_rt.lock.polarity = 0;
            g_rt.lock.r_target_q15 = g_rt.acquire.scan.r_target_q15;
            g_rt.lock.r_now_q15 = 0U;
            g_rt.lock.error_q15 = 0;
            g_rt.lock.capture_raw = 0U;
            g_rt.lock.output_raw = APPRTLOOP_GetLastRaw();
            g_rt.lock.resonance_freq_hz = 0UL;
            g_rt.lock.fn_hz = 0UL;

            if (g_rt.acquire.scan.contrast_q15 < APPACQ_CONTRAST_MIN_Q15) {
                APP_SetFault(APP_FAULT_CONTRAST);
                return;
            }

            if (!APPLOCK_StartSoft(g_rt.acquire.offset.iout_offset_raw,
                                   g_rt.acquire.offset.iref_offset_raw,
                                   &g_rt.acquire.scan)) {
                APP_SetFault(APP_FAULT_LOCK);
                return;
            }

            APPACQ_GotoStep(ACQ_STEP_SOFTLOCK_RUN);
            break;

        case ACQ_STEP_SOFTLOCK_RUN:
            if (APPRTLOOP_HasError() || APPLOCK_HasError()) {
                APPLOCK_Stop();
                APP_SetFault(APP_FAULT_LOCK);
                return;
            }

            lock_result = APPLOCK_GetResult();
            if (lock_result == NULL) {
                break;
            }

            g_rt.lock = *lock_result;

            if (g_rt.lock.soft_locked) {
                APPACQ_GotoStep(ACQ_STEP_RESONANCE_PREP);
            }
            break;

        case ACQ_STEP_RESONANCE_PREP:
            if (!APPLOCK_StartResonanceSweep()) {
                APPLOCK_Stop();
                APP_SetFault(APP_FAULT_LOCK);
                return;
            }

            APPACQ_GotoStep(ACQ_STEP_RESONANCE_RUN);
            break;

        case ACQ_STEP_RESONANCE_RUN:
            if (APPRTLOOP_HasError() || APPLOCK_HasError()) {
                APPLOCK_Stop();
                APP_SetFault(APP_FAULT_LOCK);
                return;
            }

            lock_result = APPLOCK_GetResult();
            if (lock_result == NULL) {
                break;
            }

            g_rt.lock = *lock_result;
            if (g_rt.lock.resonance_done) {
                APPACQ_GotoStep(ACQ_STEP_RESONANCE_DONE);
            }
            break;

        case ACQ_STEP_RESONANCE_DONE:
            if (APPACQ_StepElapsed(APPACQ_SUMMARY_HOLD_MS)) {
                APP_GotoPage(APP_PAGE_LOCK);
            }
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
    uint32_t contrast_permille;

    switch (g_rt.acquire.step) {
        case ACQ_STEP_OFFSET_DONE:
            APPW_DrawFrame("Offset Zero");

            (void)snprintf(line, sizeof(line), "Iout: %4u", (unsigned)g_rt.acquire.offset.iout_offset_raw);
            APPW_WriteBodyLine(22, line);

            (void)snprintf(line, sizeof(line), "Iref: %4u", (unsigned)g_rt.acquire.offset.iref_offset_raw);
            APPW_WriteBodyLine(36, line);
            APPW_WriteBodyLine(54, "Turn laser on");
            APPW_WriteBodyLine(68, "Short press scan");
            break;

        case ACQ_STEP_SCAN_DONE:
            APPW_DrawFrame("Scan Result");
            contrast_permille = ((uint32_t)g_rt.acquire.scan.contrast_q15 * 1000U + 16384U) >> 15;

            (void)snprintf(line, sizeof(line), "Ctr:%3u.%1u%% %u/%u",
                           (unsigned)(contrast_permille / 10U),
                           (unsigned)(contrast_permille % 10U),
                           (unsigned)g_rt.acquire.scan.cycles_done,
                           (unsigned)g_rt.acquire.scan.cycles_total);
            APPW_WriteBodyLine(18, line);

            (void)snprintf(line, sizeof(line), "Rmax: %lu", (unsigned long)g_rt.acquire.scan.r_max_q15);
            APPW_WriteBodyLine(32, line);

            (void)snprintf(line, sizeof(line), "Rmin: %lu", (unsigned long)g_rt.acquire.scan.r_min_q15);
            APPW_WriteBodyLine(46, line);

            (void)snprintf(line, sizeof(line), "Rtgt: %lu", (unsigned long)g_rt.acquire.scan.r_target_q15);
            APPW_WriteBodyLine(60, line);
            APPW_WriteBodyLine(74, "Auto locking...");
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

        case ACQ_STEP_SOFTLOCK_PREP:
        case ACQ_STEP_SOFTLOCK_RUN:
            APPW_DrawFrame("Resonance Scan");
            APPW_WriteBodyLine(24, "Preparing soft lock");
            APPW_WriteBodyLine(42, "Low bandwidth PI");
            APPW_WriteBodyLine(56, "Please wait...");
            break;

        case ACQ_STEP_RESONANCE_PREP:
        case ACQ_STEP_RESONANCE_RUN:
            APPW_DrawFrame("Resonance Scan");
            APPW_WriteBodyLine(24, "Sweeping...");
            APPW_WriteBodyLine(42, "Soft lock hold");
            APPW_WriteBodyLine(56, "IQ measuring...");
            break;

        case ACQ_STEP_RESONANCE_DONE:
            APPW_DrawFrame("Resonance");
            (void)snprintf(line, sizeof(line), "Fn: %lu.%1lu kHz",
                           (unsigned long)(g_rt.lock.fn_hz / 1000UL),
                           (unsigned long)((g_rt.lock.fn_hz % 1000UL) / 100UL));
            APPW_WriteBodyLine(24, line);
            APPW_WriteBodyLine(42, "Q : --");
            APPW_WriteBodyLine(58, "Entering lock...");
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

    APPACQ_GotoStep(ACQ_STEP_SCAN_PREP);
}

static void APPACQ_GotoStep(AcquireStep_t step)
{
    g_rt.acquire.step = step;
    s_step_enter_ms = HAL_GetTick();
    APP_RequestRender();
}

static bool APPACQ_StepElapsed(uint32_t elapsed_ms)
{
    return ((uint32_t)(HAL_GetTick() - s_step_enter_ms) >= elapsed_ms);
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
