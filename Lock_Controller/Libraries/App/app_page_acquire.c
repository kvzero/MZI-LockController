#include "app_page_acquire.h"
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
#define ST7735_GREEN_TITLE_BG ST7735_COLOR565(0, 165, 64)

typedef enum {
    APPACQ_FLOW_IDLE = 0,
    APPACQ_FLOW_OFFSET_RUN,
    APPACQ_FLOW_OFFSET_DONE,
    APPACQ_FLOW_SCAN_PREP,
    APPACQ_FLOW_SCAN_RUN,
    APPACQ_FLOW_SCAN_DONE,
    APPACQ_FLOW_SOFTLOCK_RUN,
    APPACQ_FLOW_RESONANCE_RUN,
    APPACQ_FLOW_RESONANCE_DONE,
} AppAcquireFlowStage_t;

typedef struct {
    uint16_t iout_offset_raw;
    uint16_t iref_offset_raw;
    bool     valid;
} AppOffsetResult_t;

typedef struct {
    AppAcquireFlowStage_t flow_stage;
    AppOffsetResult_t     offset;
} AppAcquireState_t;

static void APPACQ_Enter(void);
static void APPACQ_Process(void);
static void APPACQ_Render(void);
static void APPACQ_OnButton(Button_Event_t evt);

static void APPACQ_GotoFlowStage(AppAcquireFlowStage_t flow_stage);
static bool APPACQ_FlowStageElapsed(uint32_t elapsed_ms);
static bool APPACQ_ReadOffsetAverage(uint16_t *iout, uint16_t *iref);

static AppAcquireState_t s_acquire;
static uint32_t s_flow_stage_enter_ms;

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
    s_acquire = (AppAcquireState_t){
        .flow_stage = APPACQ_FLOW_IDLE,
    };
    APPACQ_GotoFlowStage(APPACQ_FLOW_OFFSET_RUN);
}

static void APPACQ_Process(void)
{
    const AppLockRuntime_t *lock_result;
    const AppScanResult_t *scan_result;

    switch (s_acquire.flow_stage) {
        case APPACQ_FLOW_OFFSET_RUN:
            if (!APPACQ_ReadOffsetAverage(&s_acquire.offset.iout_offset_raw,
                                          &s_acquire.offset.iref_offset_raw)) {
                APP_SetFault(APP_FAULT_ADC);
                return;
            }

            s_acquire.offset.valid = true;
            APPACQ_GotoFlowStage(APPACQ_FLOW_OFFSET_DONE);
            break;

        case APPACQ_FLOW_SCAN_PREP:
            if (!APPSCAN_Start(APPACQ_SCAN_CYCLES,
                               s_acquire.offset.iout_offset_raw,
                               s_acquire.offset.iref_offset_raw)) {
                APP_SetFault(APP_FAULT_ADC);
                return;
            }

            APPACQ_GotoFlowStage(APPACQ_FLOW_SCAN_RUN);
            break;

        case APPACQ_FLOW_SCAN_RUN:
            if (APPRTLOOP_HasError()) {
                APP_SetFault(APP_FAULT_ADC);
                return;
            }

            scan_result = APPSCAN_GetResult();
            if (!APPSCAN_IsDone() || (scan_result == NULL) || !scan_result->valid) {
                break;
            }

            APPACQ_GotoFlowStage(APPACQ_FLOW_SCAN_DONE);
            break;

        case APPACQ_FLOW_SCAN_DONE:
            if (APPACQ_FlowStageElapsed(APPACQ_SUMMARY_HOLD_MS)) {
                scan_result = APPSCAN_GetResult();
                if ((scan_result == NULL) || !scan_result->valid) {
                    APP_SetFault(APP_FAULT_LOCK);
                    return;
                }

                if (scan_result->contrast_q15 < APPACQ_CONTRAST_MIN_Q15) {
                    APP_SetFault(APP_FAULT_CONTRAST);
                    return;
                }

                if (!APPLOCK_StartSoft(s_acquire.offset.iout_offset_raw,
                                       s_acquire.offset.iref_offset_raw,
                                       scan_result)) {
                    APP_SetFault(APP_FAULT_LOCK);
                    return;
                }

                APPACQ_GotoFlowStage(APPACQ_FLOW_SOFTLOCK_RUN);
            }
            break;

        case APPACQ_FLOW_SOFTLOCK_RUN:
            if (APPRTLOOP_HasError() || APPLOCK_HasError()) {
                APPLOCK_Stop();
                APP_SetFault(APP_FAULT_LOCK);
                return;
            }

            lock_result = APPLOCK_GetResult();
            if (lock_result == NULL) {
                break;
            }

            if (lock_result->stage == APP_LOCK_STAGE_SOFT) {
                if (!APPLOCK_StartResonanceSweep()) {
                    APPLOCK_Stop();
                    APP_SetFault(APP_FAULT_LOCK);
                    return;
                }

                s_acquire.flow_stage = APPACQ_FLOW_RESONANCE_RUN;
                s_flow_stage_enter_ms = HAL_GetTick();
            }
            break;

        case APPACQ_FLOW_RESONANCE_RUN:
            if (APPRTLOOP_HasError() || APPLOCK_HasError()) {
                APPLOCK_Stop();
                APP_SetFault(APP_FAULT_LOCK);
                return;
            }

            lock_result = APPLOCK_GetResult();
            if (lock_result == NULL) {
                break;
            }

            if ((lock_result->stage == APP_LOCK_STAGE_SOFT) &&
                (lock_result->fn_hz != 0UL)) {
                APPACQ_GotoFlowStage(APPACQ_FLOW_RESONANCE_DONE);
            }
            break;

        case APPACQ_FLOW_RESONANCE_DONE:
            if (APPACQ_FlowStageElapsed(APPACQ_SUMMARY_HOLD_MS)) {
                APP_GotoPage(APP_PAGE_LOCK);
            }
            break;

        case APPACQ_FLOW_OFFSET_DONE:
        case APPACQ_FLOW_IDLE:
        default:
            break;
    }
}

static void APPACQ_Render(void)
{
    char line[32];
    uint32_t contrast_permille;
    const AppLockRuntime_t *lock_result;
    const AppScanResult_t *scan_result;

    if (g_hw->hlcd != NULL) {
        ST7735_FillScreen(g_hw->hlcd, ST7735_BLACK);
    }

    lock_result = APPLOCK_GetResult();
    scan_result = APPSCAN_GetResult();

    switch (s_acquire.flow_stage) {
        case APPACQ_FLOW_OFFSET_DONE:
            APPW_DrawFrame("Offset Zero", ST7735_WHITE, ST7735_GREEN_TITLE_BG);

            (void)snprintf(line, sizeof(line), "Iout:%4u Iref:%4u",
                           (unsigned)s_acquire.offset.iout_offset_raw,
                           (unsigned)s_acquire.offset.iref_offset_raw);
            APPW_WriteBodyLine(34, line, ST7735_YELLOW);
            APPW_WriteBodyLine(50, "Turn laser on", ST7735_WHITE);
            APPW_WriteBodyLine(64, "Short press to scan", ST7735_WHITE);
            break;

        case APPACQ_FLOW_OFFSET_RUN:
        case APPACQ_FLOW_IDLE:
        default:
            APPW_DrawFrame("Offset Zero", ST7735_BLACK, ST7735_WHITE);
            APPW_WriteBodyLine(38, "Sampling PD offsets", ST7735_WHITE);
            APPW_WriteBodyLine(54, "Please wait...", ST7735_WHITE);
            break;

        case APPACQ_FLOW_SCAN_PREP:
        case APPACQ_FLOW_SCAN_RUN:
            APPW_DrawFrame("Fringe Scan", ST7735_BLACK, ST7735_WHITE);

            (void)snprintf(line, sizeof(line), "Cycles: %u",
                           (unsigned)(((scan_result != NULL) && (scan_result->cycles_total != 0U)) ? scan_result->cycles_total : APPACQ_SCAN_CYCLES));
            APPW_WriteBodyLine(34, line, ST7735_WHITE);
            APPW_WriteBodyLine(49, "Laser on, scanning", ST7735_WHITE);
            APPW_WriteBodyLine(64, "Please wait...", ST7735_WHITE);
            break;

        case APPACQ_FLOW_SCAN_DONE:
            APPW_DrawFrame("Scan Result", ST7735_WHITE, ST7735_GREEN_TITLE_BG);
            contrast_permille = 0U;
            if ((scan_result != NULL) && scan_result->valid) {
                contrast_permille = ((uint32_t)scan_result->contrast_q15 * 1000U + 16384U) >> 15;
            }

            (void)snprintf(line, sizeof(line), "Ctr:%3u.%1u%%",
                           (unsigned)(contrast_permille / 10U),
                           (unsigned)(contrast_permille % 10U));
            APPW_WriteBodyLine(34, line, ST7735_YELLOW);
            APPW_WriteBodyLine(49, "Lock point found", ST7735_WHITE);
            APPW_WriteBodyLine(64, "Preparing soft lock", ST7735_WHITE);
            break;

        case APPACQ_FLOW_SOFTLOCK_RUN:
        case APPACQ_FLOW_RESONANCE_RUN:
            APPW_DrawFrame("Resonance Scan", ST7735_BLACK, ST7735_WHITE);
            APPW_WriteBodyLine(34, "Sweeping...", ST7735_WHITE);
            APPW_WriteBodyLine(49, "Soft lock hold", ST7735_WHITE);
            APPW_WriteBodyLine(64, "IQ measuring...", ST7735_WHITE);
            break;

        case APPACQ_FLOW_RESONANCE_DONE:
            APPW_DrawFrame("Resonance", ST7735_WHITE, ST7735_GREEN_TITLE_BG);
            (void)snprintf(line, sizeof(line), "Fn: %lu.%1lu kHz",
                           (unsigned long)(((lock_result != NULL) ? lock_result->fn_hz : 0UL) / 1000UL),
                           (unsigned long)((((lock_result != NULL) ? lock_result->fn_hz : 0UL) % 1000UL) / 100UL));
            APPW_WriteBodyLine(34, line, ST7735_YELLOW);
            APPW_WriteBodyLine(49, "Q : --", ST7735_WHITE);
            APPW_WriteBodyLine(64, "Entering lock...", ST7735_WHITE);
            break;

    }
}

static void APPACQ_OnButton(Button_Event_t evt)
{
    if ((evt.id == BTN_ID_NONE) || (evt.action != BTN_ACT_SHORT)) {
        return;
    }

    if (s_acquire.flow_stage != APPACQ_FLOW_OFFSET_DONE) {
        return;
    }

    APPACQ_GotoFlowStage(APPACQ_FLOW_SCAN_PREP);
}

static void APPACQ_GotoFlowStage(AppAcquireFlowStage_t flow_stage)
{
    s_acquire.flow_stage = flow_stage;
    s_flow_stage_enter_ms = HAL_GetTick();
    APP_RequestRender();
}

static bool APPACQ_FlowStageElapsed(uint32_t elapsed_ms)
{
    return ((uint32_t)(HAL_GetTick() - s_flow_stage_enter_ms) >= elapsed_ms);
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
