#include "app_rtloop.h"

#include "app_hardware.h"
#include "app_lock.h"
#include "app_scan.h"

typedef struct {
    volatile AppRtloopMode_t mode;
    volatile bool            active;
    volatile bool            error;

    uint32_t dma_cdr_word;

    uint16_t iout_offset_raw;
    uint16_t iref_offset_raw;
    uint16_t last_dac_raw;
} AppRtloopState_t;

static AppRtloopState_t s_rtloop;

static bool APPRTLOOP_Start(AppRtloopMode_t mode,
                            uint16_t iout_offset_raw,
                            uint16_t iref_offset_raw);
static void APPRTLOOP_OnDone(const uint32_t *cdr_buf,
                             uint16_t frame_count,
                             void *user_ctx);
static void APPRTLOOP_OnError(void *user_ctx);

void APPRTLOOP_Init(void)
{
    s_rtloop.mode = APP_RTLOOP_MODE_IDLE;
    s_rtloop.active = false;
    s_rtloop.error = false;
    s_rtloop.dma_cdr_word = 0U;
    s_rtloop.iout_offset_raw = 0U;
    s_rtloop.iref_offset_raw = 0U;
    s_rtloop.last_dac_raw = 0U;
}

bool APPRTLOOP_StartScan(uint16_t iout_offset_raw, uint16_t iref_offset_raw)
{
    return APPRTLOOP_Start(APP_RTLOOP_MODE_SCAN, iout_offset_raw, iref_offset_raw);
}

bool APPRTLOOP_StartLock(uint16_t iout_offset_raw, uint16_t iref_offset_raw)
{
    return APPRTLOOP_Start(APP_RTLOOP_MODE_LOCK, iout_offset_raw, iref_offset_raw);
}

void APPRTLOOP_Stop(void)
{
    if (s_rtloop.active && (g_hw != NULL) && (g_hw->hpdadc != NULL)) {
        PDADC_StopContinuous(g_hw->hpdadc);
    }

    s_rtloop.active = false;
    s_rtloop.mode = APP_RTLOOP_MODE_IDLE;
}

bool APPRTLOOP_IsActive(void)
{
    return (bool)s_rtloop.active;
}

bool APPRTLOOP_HasError(void)
{
    return (bool)s_rtloop.error;
}

AppRtloopMode_t APPRTLOOP_GetMode(void)
{
    return s_rtloop.mode;
}

void APPRTLOOP_WriteRaw(uint16_t raw)
{
    if ((g_hw == NULL) || (g_hw->hvdac == NULL)) {
        return;
    }

    HVDAC_WriteRaw(g_hw->hvdac, raw);
    s_rtloop.last_dac_raw = raw;
}

uint16_t APPRTLOOP_GetLastRaw(void)
{
    return s_rtloop.last_dac_raw;
}

static bool APPRTLOOP_Start(AppRtloopMode_t mode,
                            uint16_t iout_offset_raw,
                            uint16_t iref_offset_raw)
{
    if ((mode == APP_RTLOOP_MODE_IDLE) ||
        s_rtloop.active ||
        (g_hw == NULL) ||
        (g_hw->hpdadc == NULL)) {
        return false;
    }

    s_rtloop.mode = mode;
    s_rtloop.active = true;
    s_rtloop.error = false;
    s_rtloop.dma_cdr_word = 0U;
    s_rtloop.iout_offset_raw = iout_offset_raw;
    s_rtloop.iref_offset_raw = iref_offset_raw;

    if (!PDADC_StartContinuous(g_hw->hpdadc,
                               &s_rtloop.dma_cdr_word,
                               1U,
                               APPRTLOOP_OnDone,
                               APPRTLOOP_OnError,
                               NULL)) {
        s_rtloop.mode = APP_RTLOOP_MODE_IDLE;
        s_rtloop.active = false;
        return false;
    }

    return true;
}

static void APPRTLOOP_OnDone(const uint32_t *cdr_buf,
                             uint16_t frame_count,
                             void *user_ctx)
{
    uint16_t i;
    PDADC_Frame_t frame;
    AppRtloopSample_t sample;

    (void)user_ctx;

    if ((cdr_buf == NULL) || (frame_count == 0U) || !s_rtloop.active) {
        return;
    }

    for (i = 0U; i < frame_count; ++i) {
        frame = PDADC_UnpackCDR(cdr_buf[i]);

        /*
         * App-level channel convention:
         *   pd_a -> interferometer output channel (Iout)
         *   pd_b -> laser-reference channel      (Iref)
         *
         * Keep this mapping centralized here so scan/lock code only deals with
         * app semantics instead of raw ADC channel names.
         */
        sample.iout_raw = frame.pd_a;
        sample.iref_raw = frame.pd_b;
        sample.iout = (int32_t)sample.iout_raw - (int32_t)s_rtloop.iout_offset_raw;
        sample.iref = (int32_t)sample.iref_raw - (int32_t)s_rtloop.iref_offset_raw;

        switch (s_rtloop.mode) {
            case APP_RTLOOP_MODE_SCAN:
                APPSCAN_OnSample(&sample);
                break;

            case APP_RTLOOP_MODE_LOCK:
                APPLOCK_OnSample(&sample);
                break;

            case APP_RTLOOP_MODE_IDLE:
            default:
                break;
        }
    }
}

static void APPRTLOOP_OnError(void *user_ctx)
{
    (void)user_ctx;

    s_rtloop.error = true;
    s_rtloop.active = false;
    s_rtloop.mode = APP_RTLOOP_MODE_IDLE;
}
