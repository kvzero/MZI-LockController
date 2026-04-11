#include "app_page_lock.h"
#include <stdio.h>
#include "stm32g4xx_hal.h"
#include "app.h"
#include "app_core.h"
#include "app_hardware.h"
#include "app_lock.h"
#include "app_rtloop.h"
#include "app_widgets.h"

#define APPLOCK_PAGE_REFRESH_MS 100U

static void APPPAGELOCK_Enter(void);
static void APPPAGELOCK_Process(void);
static void APPPAGELOCK_Render(void);

static uint32_t s_last_render_ms;

const AppPageOps_t APP_PAGE_LOCK_OPS = {
    .enter = APPPAGELOCK_Enter,
    .process = APPPAGELOCK_Process,
    .render = APPPAGELOCK_Render,
};

static void APPPAGELOCK_Enter(void)
{
    if (!APPLOCK_StartHardLock()) {
        APP_SetFault(APP_FAULT_LOCK);
        return;
    }

    s_last_render_ms = HAL_GetTick();
    APP_RequestRender();
}

static void APPPAGELOCK_Process(void)
{
    uint32_t now_ms;

    if (APPRTLOOP_HasError() || APPLOCK_HasError()) {
        APPLOCK_Stop();
        APP_SetFault(APP_FAULT_LOCK);
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_last_render_ms) >= APPLOCK_PAGE_REFRESH_MS) {
        s_last_render_ms = now_ms;
        APP_RequestRender();
    }
}

static void APPPAGELOCK_Render(void)
{
    char line[32];
    const char *status;
    const AppLockRuntime_t *lock_result;
    uint32_t out_mv;
    uint16_t output_raw;
    uint32_t max_mv;
    uint16_t max_raw;

    APPW_DrawFrame("Lock");
    lock_result = APPLOCK_GetResult();
    output_raw = APPRTLOOP_GetLastRaw();

    (void)snprintf(line, sizeof(line), "Fn: %lu.%1lu kHz",
                   (unsigned long)(((lock_result != NULL) ? lock_result->fn_hz : 0UL) / 1000UL),
                   (unsigned long)((((lock_result != NULL) ? lock_result->fn_hz : 0UL) % 1000UL) / 100UL));
    APPW_WriteBodyLine(22, line);

    out_mv = 0UL;
    if ((g_hw->hvdac != NULL) && (g_hw->hvdac->max_raw != 0U)) {
        max_raw = g_hw->hvdac->max_raw;
        max_mv = (uint32_t)(g_hw->hvdac->max_voltage * 1000.0f + 0.5f);
        out_mv = (uint32_t)(((uint64_t)output_raw * max_mv + (max_raw / 2U)) / max_raw);
    }

    (void)snprintf(line, sizeof(line), "Out:%3lu.%03lu V",
                   (unsigned long)(out_mv / 1000UL),
                   (unsigned long)(out_mv % 1000UL));
    APPW_WriteBodyLine(40, line);

    status = "Status: Idle";
    if ((lock_result != NULL) && (lock_result->stage == APP_LOCK_STAGE_HARD)) {
        status = "Status: Hard lock";
    } else if ((lock_result != NULL) &&
               ((lock_result->stage == APP_LOCK_STAGE_SOFT) ||
                (lock_result->stage == APP_LOCK_STAGE_CAPTURE) ||
                (lock_result->stage == APP_LOCK_STAGE_RESONANCE))) {
        status = "Status: Soft hold";
    } else if ((lock_result != NULL) && (lock_result->stage == APP_LOCK_STAGE_FAULT)) {
        status = "Status: Fault";
    }

    APPW_WriteBodyLine(58, status);
}
