#include "app_page_lock.h"
#include <stdio.h>
#include "stm32g4xx_hal.h"
#include "app.h"
#include "app_core.h"
#include "app_hardware.h"
#include "app_lock.h"
#include "app_rtloop.h"
#include "app_scan.h"
#include "app_widgets.h"

#define APPLOCK_PAGE_REFRESH_MS 100U

static const int16_t APPLOCK_BAR_X = 10;
static const int16_t APPLOCK_BAR_Y = 66;
static const int16_t APPLOCK_BAR_W = 135;
static const int16_t APPLOCK_BAR_H = 10;

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
    char line[32];
    ST7735_Handle_t *lcd;
    const AppLockRuntime_t *lock_result;
    const AppScanResult_t *scan_result;
    uint32_t contrast_permille = 0U;

    if (!APPLOCK_StartHardLock()) {
        APP_SetFault(APP_FAULT_LOCK);
        return;
    }

    lcd = g_hw->hlcd;
    if (lcd == NULL) {
        return;
    }

    ST7735_FillScreen(lcd, ST7735_BLACK);
    APPW_DrawFrame("Lock", ST7735_BLACK, ST7735_WHITE);

    lock_result = APPLOCK_GetResult();
    scan_result = APPSCAN_GetResult();

    if ((scan_result != NULL) && scan_result->valid) {
        contrast_permille = ((uint32_t)scan_result->contrast_q15 * 1000U + 16384U) >> 15;
    }

    ST7735_FillRectangle(lcd, 8, 34, ST7735_WIDTH - 8, 10, ST7735_BLACK);
    (void)snprintf(line, sizeof(line), "Ctr:%2lu.%1lu%%  Fn:%lu.%1luk",
                   (unsigned long)(contrast_permille / 10U),
                   (unsigned long)(contrast_permille % 10U),
                   (unsigned long)(((lock_result != NULL) ? lock_result->fn_hz : 0UL) / 1000UL),
                   (unsigned long)((((lock_result != NULL) ? lock_result->fn_hz : 0UL) % 1000UL) / 100UL));
    APPW_WriteBodyLine(34, line, ST7735_WHITE);

    s_last_render_ms = HAL_GetTick();
    APP_RequestRender();
}

static void APPPAGELOCK_Process(void)
{
    uint32_t now_ms;

    if (APPLOCK_HasRefLow()) {
        APPLOCK_Stop();
        APP_SetFault(APP_FAULT_REF_LOW);
        return;
    }

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
    ST7735_Handle_t *lcd;
    uint32_t bar_fill_px;
    uint32_t out_mv;
    uint16_t output_raw;
    uint32_t max_mv = 0UL;
    uint16_t max_raw = 0U;

    lcd = g_hw->hlcd;
    if (lcd == NULL) {
        return;
    }

    output_raw = APPRTLOOP_GetLastRaw();
    out_mv = 0UL;
    if ((g_hw->hvdac != NULL) && (g_hw->hvdac->max_raw != 0U)) {
        max_raw = g_hw->hvdac->max_raw;
        max_mv = (uint32_t)(g_hw->hvdac->max_voltage * 1000.0f + 0.5f);
        out_mv = (uint32_t)(((uint64_t)output_raw * max_mv + (max_raw / 2U)) / max_raw);
    }

    bar_fill_px = 0UL;
    if (max_raw != 0U) {
        if (output_raw > max_raw) {
            output_raw = max_raw;
        }

        bar_fill_px = ((uint32_t)output_raw * (uint32_t)APPLOCK_BAR_W) / (uint32_t)max_raw;
    }

    (void)snprintf(line, sizeof(line), "Out:%3lu.%03lu V",
                   (unsigned long)(out_mv / 1000UL),
                   (unsigned long)(out_mv % 1000UL));
    APPW_WriteBodyLine(49, line, ST7735_WHITE);

    ST7735_FillRectangle(lcd,
                         APPLOCK_BAR_X,
                         APPLOCK_BAR_Y,
                         APPLOCK_BAR_W,
                         APPLOCK_BAR_H,
                         ST7735_BLACK);
    if (bar_fill_px != 0UL) {
        ST7735_FillRectangle(lcd,
                             APPLOCK_BAR_X,
                             APPLOCK_BAR_Y,
                             (int16_t)bar_fill_px,
                             APPLOCK_BAR_H,
                             ST7735_WHITE);
    }
    ST7735_DrawRectangle(lcd,
                         APPLOCK_BAR_X,
                         APPLOCK_BAR_Y,
                         APPLOCK_BAR_W,
                         APPLOCK_BAR_H,
                         1,
                         ST7735_WHITE);
}
