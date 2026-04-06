#include "app_scan.h"

#include "app_hardware.h"

#define APPSCAN_MEDIAN_WINDOW  9U

typedef struct {
    bool     active;
    bool     done;

    uint8_t  cycles_done;
    uint8_t  cycles_total;
    uint8_t  edge_count;

    uint16_t dac_raw;
    uint16_t dac_max_raw;
    int8_t   direction;

    uint32_t median_buf[APPSCAN_MEDIAN_WINDOW];
    uint8_t  median_count;
    uint8_t  median_index;

    uint32_t r_max_q15;
    uint32_t r_min_q15;

    AppScanResult_t result;
} AppScanState_t;

static AppScanState_t s_scan;

static void APPSCAN_ClearState(void);
static uint32_t APPSCAN_ComputeRatioQ15(const AppRtloopSample_t *sample);
static bool APPSCAN_PushMedian(uint32_t value, uint32_t *median_out);
static uint32_t APPSCAN_Median9(const uint32_t *values);
static void APPSCAN_AdvanceWaveform(void);
static void APPSCAN_Finish(void);

void APPSCAN_Reset(void)
{
    APPSCAN_ClearState();
}

bool APPSCAN_Start(uint8_t cycles_total,
                   uint16_t iout_offset_raw,
                   uint16_t iref_offset_raw)
{
    if ((cycles_total == 0U) ||
        s_scan.active ||
        (g_hw == NULL) ||
        (g_hw->hvdac == NULL) ||
        (g_hw->hpdadc == NULL) ||
        (g_hw->hvdac->max_raw == 0U)) {
        return false;
    }

    APPSCAN_ClearState();

    s_scan.active = true;
    s_scan.cycles_total = cycles_total;
    s_scan.dac_max_raw = g_hw->hvdac->max_raw;
    s_scan.direction = 1;
    s_scan.r_min_q15 = 0xFFFFFFFFUL;
    s_scan.result.cycles_total = cycles_total;

    APPRTLOOP_WriteRaw(0U);

    if (!APPRTLOOP_StartScan(iout_offset_raw, iref_offset_raw)) {
        APPSCAN_ClearState();
        return false;
    }

    return true;
}

void APPSCAN_Stop(void)
{
    if (s_scan.active) {
        APPRTLOOP_Stop();
    }

    s_scan.active = false;
}

bool APPSCAN_IsActive(void)
{
    return s_scan.active;
}

bool APPSCAN_IsDone(void)
{
    return s_scan.done;
}

const AppScanResult_t *APPSCAN_GetResult(void)
{
    return &s_scan.result;
}

void APPSCAN_OnSample(const AppRtloopSample_t *sample)
{
    uint32_t r_q15;
    uint32_t r_med_q15;

    if (!s_scan.active || (sample == NULL)) {
        return;
    }

    r_q15 = APPSCAN_ComputeRatioQ15(sample);

    if (APPSCAN_PushMedian(r_q15, &r_med_q15)) {
        if (r_med_q15 > s_scan.r_max_q15) {
            s_scan.r_max_q15 = r_med_q15;
        }

        if (r_med_q15 < s_scan.r_min_q15) {
            s_scan.r_min_q15 = r_med_q15;
        }
    }

    APPSCAN_AdvanceWaveform();
}

static void APPSCAN_ClearState(void)
{
    uint8_t i;

    s_scan.active = false;
    s_scan.done = false;
    s_scan.cycles_done = 0U;
    s_scan.cycles_total = 0U;
    s_scan.edge_count = 0U;
    s_scan.dac_raw = 0U;
    s_scan.dac_max_raw = 0U;
    s_scan.direction = 1;
    s_scan.median_count = 0U;
    s_scan.median_index = 0U;
    s_scan.r_max_q15 = 0U;
    s_scan.r_min_q15 = 0U;

    for (i = 0U; i < APPSCAN_MEDIAN_WINDOW; ++i) {
        s_scan.median_buf[i] = 0UL;
    }

    s_scan.result.cycles_done = 0U;
    s_scan.result.cycles_total = 0U;
    s_scan.result.contrast_q15 = 0U;
    s_scan.result.r_max_q15 = 0U;
    s_scan.result.r_min_q15 = 0U;
    s_scan.result.r_target_q15 = 0U;
    s_scan.result.valid = false;
}

static uint32_t APPSCAN_ComputeRatioQ15(const AppRtloopSample_t *sample)
{
    uint32_t iout_u;
    uint32_t iref_u;

    /*
     * Scan mode currently assumes illuminated operation, so the post-offset
     * reference channel remains positive and non-zero and can be used directly
     * here. This is an agreed first-cut assumption for scan mode; fault/guard
     * handling can be added later once the scan path itself is validated.
     */
    iout_u = (uint32_t)sample->iout;
    iref_u = (uint32_t)sample->iref;

    return (uint32_t)((iout_u << 15) / iref_u);
}

static bool APPSCAN_PushMedian(uint32_t value, uint32_t *median_out)
{
    s_scan.median_buf[s_scan.median_index] = value;
    s_scan.median_index = (uint8_t)((s_scan.median_index + 1U) % APPSCAN_MEDIAN_WINDOW);

    if (s_scan.median_count < APPSCAN_MEDIAN_WINDOW) {
        s_scan.median_count++;
    }

    if (s_scan.median_count < APPSCAN_MEDIAN_WINDOW) {
        return false;
    }

    *median_out = APPSCAN_Median9(s_scan.median_buf);
    return true;
}

static uint32_t APPSCAN_Median9(const uint32_t *values)
{
    uint32_t sorted[APPSCAN_MEDIAN_WINDOW];
    uint32_t key;
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < APPSCAN_MEDIAN_WINDOW; ++i) {
        sorted[i] = values[i];
    }

    for (i = 1U; i < APPSCAN_MEDIAN_WINDOW; ++i) {
        key = sorted[i];
        j = i;

        while ((j > 0U) && (sorted[j - 1U] > key)) {
            sorted[j] = sorted[j - 1U];
            j--;
        }

        sorted[j] = key;
    }

    return sorted[APPSCAN_MEDIAN_WINDOW / 2U];
}

static void APPSCAN_AdvanceWaveform(void)
{
    /*
     * Full-range triangle scan with 1-raw-code steps.
     *
     * Current boundary policy:
     *   - Start from raw 0 and sweep upward.
     *   - Hold both endpoints as valid scan positions.
     *   - After touching max_raw, reverse direction and continue from max_raw-1.
     *   - After touching 0 on the way back, one full cycle is complete.
     *   - If more cycles remain, continue upward from raw 1.
     *
     * This keeps the scan monotonic between edges while avoiding double-writing
     * the same endpoint code on two consecutive samples.
     */
    if (s_scan.direction > 0) {
        if (s_scan.dac_raw >= s_scan.dac_max_raw) {
            s_scan.direction = -1;
            s_scan.edge_count++;
            s_scan.cycles_done = (uint8_t)(s_scan.edge_count / 2U);
            s_scan.result.cycles_done = s_scan.cycles_done;

            if (s_scan.dac_raw > 0U) {
                s_scan.dac_raw--;
            }
        } else {
            s_scan.dac_raw++;
        }
    } else {
        if (s_scan.dac_raw == 0U) {
            s_scan.direction = 1;
            s_scan.edge_count++;
            s_scan.cycles_done = (uint8_t)(s_scan.edge_count / 2U);
            s_scan.result.cycles_done = s_scan.cycles_done;

            if (s_scan.cycles_done >= s_scan.cycles_total) {
                APPSCAN_Finish();
                return;
            }

            if (s_scan.dac_max_raw != 0U) {
                s_scan.dac_raw = 1U;
            }
        } else {
            s_scan.dac_raw--;
        }
    }

    APPRTLOOP_WriteRaw(s_scan.dac_raw);
}

static void APPSCAN_Finish(void)
{
    uint32_t contrast_den;
    uint32_t contrast_num;
    uint32_t r_sum;

    r_sum = (uint32_t)s_scan.r_max_q15 + (uint32_t)s_scan.r_min_q15;
    contrast_den = r_sum;
    contrast_num = (uint32_t)s_scan.r_max_q15 - (uint32_t)s_scan.r_min_q15;

    s_scan.result.cycles_done = s_scan.cycles_done;
    s_scan.result.cycles_total = s_scan.cycles_total;
    s_scan.result.contrast_q15 = (uint16_t)(((uint64_t)contrast_num << 15) / contrast_den);
    s_scan.result.r_max_q15 = s_scan.r_max_q15;
    s_scan.result.r_min_q15 = s_scan.r_min_q15;
    s_scan.result.r_target_q15 = (uint32_t)(r_sum / 2U);
    s_scan.result.valid = true;

    s_scan.done = true;
    s_scan.active = false;

    APPRTLOOP_Stop();
}
