#include "app_lock.h"

#include "app_hardware.h"

/*
 * Lock backend:
 *   - ramp to mid-scale, sweep down, and capture the first valid lock point
 *   - hold with soft PI while resonance sweep injects a small DDS tone
 *   - switch to hard PI with the measured main-notch frequency
 */

/* Capture/filter settings used while sweeping down to the first lock point. */
#define APPLOCK_MEDIAN_WINDOW        7U
#define APPLOCK_SLOPE_VOTE_WINDOW   32U
#define APPLOCK_SLOPE_MIN_VOTES      8U

/*
 * Soft-lock PI gains in Q8: 256 == 1.0x.
 * Current values:
 *   Kp =  64 / 256 = 0.25
 *   Ki =  12 / 256 = 0.046875
 */
#define APPLOCK_SOFT_KP_GAIN_Q8     64L
#define APPLOCK_SOFT_KI_GAIN_Q8     12L

/*
 * Hard-lock PI gains in Q8: 256 == 1.0x.
 * The hard P path uses a low-shelf error shaper:
 *   slow += (error - slow) >> LP_SHIFT
 *   p_error = slow + ((error - slow) >> HF_GAIN_SHIFT)
 * so low frequency error gets full P gain while high frequency error is
 * reduced to 1 / (2^HF_GAIN_SHIFT).
 * APPLOCK_NOTCH_BW_HZ is the approximate main-notch bandwidth in hertz. Its
 * center frequency comes from the measured resonance fn.
 */
#define APPLOCK_HARD_KP_GAIN_Q8      7400L
#define APPLOCK_HARD_KI_GAIN_Q8      900L
#define APPLOCK_NOTCH_BW_HZ          7700UL
#define APPLOCK_HARD_SHELF_LP_SHIFT       3U
#define APPLOCK_HARD_SHELF_HF_GAIN_SHIFT  3U

/*
 * Resonance sweep / IQ identification settings.
 * The first version still uses a fixed full-range coarse sweep plus a narrow
 * fine sweep around the best coarse point.
 */
#define APPLOCK_RESONANCE_INJ_AMP_RAW        64U
#define APPLOCK_RESONANCE_COARSE_START_HZ  5000UL
#define APPLOCK_RESONANCE_COARSE_END_HZ   40000UL
#define APPLOCK_RESONANCE_COARSE_STEP_HZ    500UL
#define APPLOCK_RESONANCE_COARSE_SAMPLES   1024U
#define APPLOCK_RESONANCE_FINE_SPAN_HZ     1000UL
#define APPLOCK_RESONANCE_FINE_STEP_HZ      100UL
#define APPLOCK_RESONANCE_FINE_SAMPLES     2048U
#define APPLOCK_IQ_AMP_SHIFT                 16U

/*
 * Contrast-based error normalization.
 *   APPLOCK_CONTRAST_REF_Q15   = 0.75
 *   APPLOCK_ERROR_GAIN_MIN_Q15 = 0.5x
 *   APPLOCK_ERROR_GAIN_MAX_Q15 = 2.0x
 */
#define APPLOCK_CONTRAST_REF_Q15   24576U
#define APPLOCK_ERROR_GAIN_MIN_Q15 16384UL
#define APPLOCK_ERROR_GAIN_MAX_Q15 65536UL

/* Shared realtime constants used by DDS and notch configuration. */
#define APPLOCK_SAMPLE_RATE_HZ   200000UL

/* DDS lookup-table implementation constants. */
#define APPLOCK_SINE_TABLE_SIZE      256U
#define APPLOCK_SINE_TABLE_MASK     0xFFU
#define APPLOCK_COS_TABLE_OFFSET   (APPLOCK_SINE_TABLE_SIZE / 4U)

typedef enum {
    APPLOCK_STATE_IDLE = 0,
    APPLOCK_STATE_RAMP_TO_CENTER,
    APPLOCK_STATE_SWEEP_DOWN,
    APPLOCK_STATE_SOFT,
    APPLOCK_STATE_RESONANCE,
    APPLOCK_STATE_HARD,
} AppLockState_t;

typedef enum {
    APPLOCK_RESONANCE_STAGE_IDLE = 0,
    APPLOCK_RESONANCE_STAGE_COARSE,
    APPLOCK_RESONANCE_STAGE_FINE,
} AppLockResonanceStage_t;

typedef struct {
    int32_t b1_q14;
    int32_t a1_q14;
    int32_t a2_q14;
    int32_t x1;
    int32_t x2;
    int32_t y1;
    int32_t y2;
} AppLockNotchState_t;

/*
 * Private lock engine state. These fields are updated from the realtime sample
 * path; only the summarized `result` block is copied out to g_rt for UI/process
 * use.
 */
typedef struct {
    bool           active;
    bool           error;
    AppLockState_t state;

    uint16_t max_raw;
    uint16_t mid_raw;
    uint16_t output_raw;
    uint32_t r_target_q15;

    uint32_t median_buf[APPLOCK_MEDIAN_WINDOW];
    uint8_t  median_count;
    uint8_t  median_index;

    int8_t  slope_votes[APPLOCK_SLOPE_VOTE_WINDOW];
    uint8_t slope_vote_count;
    uint8_t slope_vote_index;
    uint8_t slope_pos_votes;
    uint8_t slope_neg_votes;

    bool     prev_valid;
    uint32_t prev_r_q15;
    int32_t  prev_error_q15;
    bool     crossing_seen;
    uint16_t crossing_raw;
    uint32_t crossing_r_q15;
    int32_t  crossing_error_q15;

    int8_t  polarity;
    int32_t integrator_q8;
    bool    hard_shelf_valid;
    int32_t hard_slow_error;
    uint16_t contrast_q15;
    uint32_t span_q15;
    uint32_t error_gain_q15;

    AppLockNotchState_t notch_main;

    AppLockResonanceStage_t resonance_stage;
    uint32_t resonance_freq_hz;
    uint32_t resonance_step_hz;
    uint32_t resonance_end_hz;
    uint16_t resonance_samples_target;
    uint16_t resonance_samples_done;
    uint32_t resonance_phase;
    uint32_t resonance_phase_step;
    int64_t  resonance_i_acc;
    int64_t  resonance_q_acc;
    uint64_t resonance_best_amp2;
    uint32_t resonance_best_hz;

    AppLockRuntime_t result;
} AppLockStateStore_t;

static AppLockStateStore_t s_lock;

static void APPLOCK_ClearState(void);
static void APPLOCK_ClearFilter(void);
static uint32_t APPLOCK_ComputeRatioQ15(const AppRtloopSample_t *sample);
static bool APPLOCK_PushMedian(uint32_t value, uint32_t *median_out);
static uint32_t APPLOCK_Median(const uint32_t *values);
static void APPLOCK_ClearSlopeVotes(void);
static void APPLOCK_AddSlopeVote(int8_t vote);
static bool APPLOCK_DecidePolarity(int8_t *polarity);
static bool APPLOCK_CrossedTarget(int32_t prev_error, int32_t error);
static void APPLOCK_ProcessRampToCenter(void);
static void APPLOCK_ProcessSweepDown(uint32_t r_q15);
static void APPLOCK_EnterSoft(uint32_t r_q15, int32_t error_q15);
static void APPLOCK_ProcessSoft(uint32_t r_q15);
static int32_t APPLOCK_RunPiQ8(uint32_t r_q15, int32_t *error_out);
static void APPLOCK_ProcessHard(uint32_t r_q15);
static int32_t APPLOCK_RunHardPiQ8(uint32_t r_q15);
static int32_t APPLOCK_NormalizeControlErrorQ15(int32_t control_error);
static void APPLOCK_ConfigNotch(uint32_t freq_hz);
static void APPLOCK_ConfigNotchStage(AppLockNotchState_t *notch,
                                     uint32_t freq_hz,
                                     uint32_t bw_hz);
static int32_t APPLOCK_NotchStageStep(AppLockNotchState_t *notch, int32_t value);
static int32_t APPLOCK_NotchStep(int32_t value);
static int32_t APPLOCK_SineQ14FromPhase(uint32_t phase);
static void APPLOCK_StartResonancePoint(uint32_t freq_hz,
                                        uint32_t step_hz,
                                        uint32_t end_hz,
                                        uint16_t samples_target,
                                        AppLockResonanceStage_t stage);
static void APPLOCK_ProcessResonance(uint32_t r_q15);
static void APPLOCK_FinishResonancePoint(void);
static uint64_t APPLOCK_ComputeIqAmp2(void);
static void APPLOCK_Fail(void);
static int32_t APPLOCK_ClampQ8(int32_t value_q8);
static uint16_t APPLOCK_Q8ToRaw(int32_t value_q8);

/*
 * 256-point Q15 sine table for the resonance DDS. Cosine is the same table
 * shifted by one quarter cycle. Keeping this static table here avoids runtime
 * sin/cos cost in the realtime path.
 */
static const int16_t s_lock_sine_q15[APPLOCK_SINE_TABLE_SIZE] = {
         0,    804,   1608,   2410,   3212,   4011,   4808,   5602,
      6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
     12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,
     18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
     23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,
     27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
     30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,
     32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
     32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
     32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
     30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,
     27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
     23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,
     18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
     12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,
      6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
         0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,
     -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
    -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
    -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
    -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
    -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
    -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
    -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
    -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
    -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
    -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
    -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
    -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,
     -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804
};

void APPLOCK_Reset(void)
{
    if (s_lock.active) {
        APPRTLOOP_Stop();
    }

    APPLOCK_ClearState();
}

bool APPLOCK_StartSoft(uint16_t iout_offset_raw,
                       uint16_t iref_offset_raw,
                       const AppScanResult_t *scan_result)
{
    uint32_t span_ref_q15;

    if (s_lock.active ||
        (g_hw == NULL) ||
        (g_hw->hvdac == NULL) ||
        (g_hw->hpdadc == NULL) ||
        (scan_result == NULL) ||
        !scan_result->valid ||
        (scan_result->r_max_q15 <= scan_result->r_min_q15) ||
        (g_hw->hvdac->max_raw == 0U)) {
        return false;
    }

    APPLOCK_ClearState();

    s_lock.active = true;
    s_lock.state = APPLOCK_STATE_RAMP_TO_CENTER;
    s_lock.max_raw = g_hw->hvdac->max_raw;
    s_lock.mid_raw = (uint16_t)(s_lock.max_raw / 2U);
    s_lock.output_raw = APPRTLOOP_GetLastRaw();
      if (s_lock.output_raw > s_lock.max_raw) {
          s_lock.output_raw = s_lock.max_raw;
      }
      s_lock.r_target_q15 = scan_result->r_target_q15;
      s_lock.contrast_q15 = scan_result->contrast_q15;
      s_lock.span_q15 = scan_result->r_max_q15 - scan_result->r_min_q15;
      span_ref_q15 =
          (uint32_t)((((uint64_t)scan_result->r_target_q15 * 2ULL) *
                      (uint64_t)APPLOCK_CONTRAST_REF_Q15) >> 15);
      if (span_ref_q15 == 0UL) {
          span_ref_q15 = 1UL;
      }
      s_lock.error_gain_q15 =
          (uint32_t)(((uint64_t)span_ref_q15 << 15) / (uint64_t)s_lock.span_q15);
      if (s_lock.error_gain_q15 < APPLOCK_ERROR_GAIN_MIN_Q15) {
          s_lock.error_gain_q15 = APPLOCK_ERROR_GAIN_MIN_Q15;
      }
      if (s_lock.error_gain_q15 > APPLOCK_ERROR_GAIN_MAX_Q15) {
          s_lock.error_gain_q15 = APPLOCK_ERROR_GAIN_MAX_Q15;
      }

      s_lock.result.active = true;
      s_lock.result.r_target_q15 = scan_result->r_target_q15;
      s_lock.result.output_raw = s_lock.output_raw;
      s_lock.result.hard_locked = false;
    s_lock.result.resonance_done = false;
    s_lock.result.resonance_freq_hz = 0UL;
    s_lock.result.fn_hz = 0UL;

    if (!APPRTLOOP_StartLock(iout_offset_raw, iref_offset_raw)) {
        APPLOCK_ClearState();
        return false;
    }

    return true;
}

void APPLOCK_Stop(void)
{
    if (s_lock.active) {
        APPRTLOOP_Stop();
    }

    s_lock.active = false;
    s_lock.state = APPLOCK_STATE_IDLE;
    s_lock.result.active = false;
    s_lock.result.hard_locked = false;
}

bool APPLOCK_StartResonanceSweep(void)
{
    if (!s_lock.active ||
        (s_lock.state != APPLOCK_STATE_SOFT) ||
        !s_lock.result.soft_locked) {
        return false;
    }

    s_lock.resonance_best_amp2 = 0ULL;
    s_lock.resonance_best_hz = 0UL;
    APPLOCK_StartResonancePoint(APPLOCK_RESONANCE_COARSE_START_HZ,
                                APPLOCK_RESONANCE_COARSE_STEP_HZ,
                                APPLOCK_RESONANCE_COARSE_END_HZ,
                                APPLOCK_RESONANCE_COARSE_SAMPLES,
                                APPLOCK_RESONANCE_STAGE_COARSE);
    s_lock.state = APPLOCK_STATE_RESONANCE;
    s_lock.result.hard_locked = false;
    s_lock.result.resonance_done = false;
    s_lock.result.fn_hz = 0UL;
    return true;
}

bool APPLOCK_StartHardLock(void)
{
    if (!s_lock.active ||
        (s_lock.state != APPLOCK_STATE_SOFT) ||
        !s_lock.result.soft_locked ||
        (s_lock.result.fn_hz == 0UL)) {
        return false;
    }

    APPLOCK_ConfigNotch(s_lock.result.fn_hz);

    s_lock.integrator_q8 = (int32_t)s_lock.output_raw << 8;
    s_lock.hard_shelf_valid = false;
    s_lock.hard_slow_error = 0;
    s_lock.state = APPLOCK_STATE_HARD;
    s_lock.result.hard_locked = true;
    return true;
}

bool APPLOCK_HasError(void)
{
    return s_lock.error || s_lock.result.error;
}

const AppLockRuntime_t *APPLOCK_GetResult(void)
{
    return &s_lock.result;
}

void APPLOCK_OnSample(const AppRtloopSample_t *sample)
{
    uint32_t r_q15;

    if (!s_lock.active || (sample == NULL)) {
        return;
    }

    switch (s_lock.state) {
        case APPLOCK_STATE_RAMP_TO_CENTER:
            APPLOCK_ProcessRampToCenter();
            break;

        case APPLOCK_STATE_SWEEP_DOWN:
            r_q15 = APPLOCK_ComputeRatioQ15(sample);
            if (APPLOCK_PushMedian(r_q15, &r_q15)) {
                APPLOCK_ProcessSweepDown(r_q15);
            }
            break;

        case APPLOCK_STATE_SOFT:
            r_q15 = APPLOCK_ComputeRatioQ15(sample);
            APPLOCK_ProcessSoft(r_q15);
            break;

        case APPLOCK_STATE_RESONANCE:
            r_q15 = APPLOCK_ComputeRatioQ15(sample);
            APPLOCK_ProcessResonance(r_q15);
            break;

        case APPLOCK_STATE_HARD:
            r_q15 = APPLOCK_ComputeRatioQ15(sample);
            APPLOCK_ProcessHard(r_q15);
            break;

        case APPLOCK_STATE_IDLE:
        default:
            break;
    }
}

static void APPLOCK_ClearState(void)
{
    uint8_t i;

    s_lock.active = false;
    s_lock.error = false;
    s_lock.state = APPLOCK_STATE_IDLE;
    s_lock.max_raw = 0U;
    s_lock.mid_raw = 0U;
    s_lock.output_raw = 0U;
    s_lock.r_target_q15 = 0U;
    s_lock.median_count = 0U;
    s_lock.median_index = 0U;
    s_lock.slope_vote_count = 0U;
    s_lock.slope_vote_index = 0U;
    s_lock.slope_pos_votes = 0U;
    s_lock.slope_neg_votes = 0U;
    s_lock.prev_valid = false;
    s_lock.prev_r_q15 = 0U;
    s_lock.prev_error_q15 = 0;
    s_lock.crossing_seen = false;
    s_lock.crossing_raw = 0U;
    s_lock.crossing_r_q15 = 0U;
    s_lock.crossing_error_q15 = 0;
    s_lock.polarity = 0;
    s_lock.integrator_q8 = 0;
    s_lock.hard_shelf_valid = false;
    s_lock.hard_slow_error = 0;
    s_lock.contrast_q15 = 0U;
    s_lock.span_q15 = 0UL;
    s_lock.error_gain_q15 = 0UL;
    s_lock.notch_main.b1_q14 = 0;
    s_lock.notch_main.a1_q14 = 0;
    s_lock.notch_main.a2_q14 = 0;
    s_lock.notch_main.x1 = 0;
    s_lock.notch_main.x2 = 0;
    s_lock.notch_main.y1 = 0;
    s_lock.notch_main.y2 = 0;
    s_lock.resonance_stage = APPLOCK_RESONANCE_STAGE_IDLE;
    s_lock.resonance_freq_hz = 0UL;
    s_lock.resonance_step_hz = 0UL;
    s_lock.resonance_end_hz = 0UL;
    s_lock.resonance_samples_target = 0U;
    s_lock.resonance_samples_done = 0U;
    s_lock.resonance_phase = 0UL;
    s_lock.resonance_phase_step = 0UL;
    s_lock.resonance_i_acc = 0;
    s_lock.resonance_q_acc = 0;
    s_lock.resonance_best_amp2 = 0ULL;
    s_lock.resonance_best_hz = 0UL;

    for (i = 0U; i < APPLOCK_MEDIAN_WINDOW; ++i) {
        s_lock.median_buf[i] = 0UL;
    }

    for (i = 0U; i < APPLOCK_SLOPE_VOTE_WINDOW; ++i) {
        s_lock.slope_votes[i] = 0;
    }

    s_lock.result.active = false;
    s_lock.result.soft_locked = false;
    s_lock.result.hard_locked = false;
    s_lock.result.resonance_done = false;
    s_lock.result.error = false;
    s_lock.result.polarity = 0;
    s_lock.result.r_target_q15 = 0U;
    s_lock.result.r_now_q15 = 0U;
    s_lock.result.error_q15 = 0;
    s_lock.result.capture_raw = 0U;
    s_lock.result.output_raw = 0U;
    s_lock.result.resonance_freq_hz = 0UL;
    s_lock.result.fn_hz = 0UL;
}

static void APPLOCK_ClearFilter(void)
{
    uint8_t i;

    s_lock.median_count = 0U;
    s_lock.median_index = 0U;
    s_lock.prev_valid = false;
    s_lock.prev_r_q15 = 0U;
    s_lock.prev_error_q15 = 0;

    for (i = 0U; i < APPLOCK_MEDIAN_WINDOW; ++i) {
        s_lock.median_buf[i] = 0UL;
    }
}

static uint32_t APPLOCK_ComputeRatioQ15(const AppRtloopSample_t *sample)
{
    uint32_t iout_u;
    uint32_t iref_u;

    /*
     * Lock mode uses the same first-cut illuminated-reference assumption as
     * scan mode: post-offset Iref remains positive and non-zero. Fault/guard
     * handling should be added deliberately after the control path is tested.
     */
    iout_u = (uint32_t)sample->iout;
    iref_u = (uint32_t)sample->iref;

    return (uint32_t)((iout_u << 15) / iref_u);
}

static bool APPLOCK_PushMedian(uint32_t value, uint32_t *median_out)
{
    s_lock.median_buf[s_lock.median_index] = value;
    s_lock.median_index = (uint8_t)((s_lock.median_index + 1U) % APPLOCK_MEDIAN_WINDOW);

    if (s_lock.median_count < APPLOCK_MEDIAN_WINDOW) {
        s_lock.median_count++;
    }

    if (s_lock.median_count < APPLOCK_MEDIAN_WINDOW) {
        return false;
    }

    *median_out = APPLOCK_Median(s_lock.median_buf);
    return true;
}

static uint32_t APPLOCK_Median(const uint32_t *values)
{
    uint32_t sorted[APPLOCK_MEDIAN_WINDOW];
    uint32_t key;
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < APPLOCK_MEDIAN_WINDOW; ++i) {
        sorted[i] = values[i];
    }

    for (i = 1U; i < APPLOCK_MEDIAN_WINDOW; ++i) {
        key = sorted[i];
        j = i;

        while ((j > 0U) && (sorted[j - 1U] > key)) {
            sorted[j] = sorted[j - 1U];
            j--;
        }

        sorted[j] = key;
    }

    return sorted[APPLOCK_MEDIAN_WINDOW / 2U];
}

static void APPLOCK_ClearSlopeVotes(void)
{
    uint8_t i;

    s_lock.slope_vote_count = 0U;
    s_lock.slope_vote_index = 0U;
    s_lock.slope_pos_votes = 0U;
    s_lock.slope_neg_votes = 0U;

    for (i = 0U; i < APPLOCK_SLOPE_VOTE_WINDOW; ++i) {
        s_lock.slope_votes[i] = 0;
    }
}

static void APPLOCK_AddSlopeVote(int8_t vote)
{
    int8_t old_vote;

    if (s_lock.slope_vote_count >= APPLOCK_SLOPE_VOTE_WINDOW) {
        old_vote = s_lock.slope_votes[s_lock.slope_vote_index];
        if (old_vote > 0) {
            s_lock.slope_pos_votes--;
        } else if (old_vote < 0) {
            s_lock.slope_neg_votes--;
        }
    } else {
        s_lock.slope_vote_count++;
    }

    s_lock.slope_votes[s_lock.slope_vote_index] = vote;
    if (vote > 0) {
        s_lock.slope_pos_votes++;
    } else if (vote < 0) {
        s_lock.slope_neg_votes++;
    }

    s_lock.slope_vote_index =
        (uint8_t)((s_lock.slope_vote_index + 1U) % APPLOCK_SLOPE_VOTE_WINDOW);
}

static bool APPLOCK_DecidePolarity(int8_t *polarity)
{
    uint8_t valid_votes = (uint8_t)(s_lock.slope_pos_votes + s_lock.slope_neg_votes);

    if ((polarity == NULL) || (valid_votes < APPLOCK_SLOPE_MIN_VOTES)) {
        return false;
    }

    if (s_lock.slope_pos_votes > s_lock.slope_neg_votes) {
        *polarity = 1;
        return true;
    }

    if (s_lock.slope_neg_votes > s_lock.slope_pos_votes) {
        *polarity = -1;
        return true;
    }

    return false;
}

static bool APPLOCK_CrossedTarget(int32_t prev_error, int32_t error)
{
    if ((prev_error == 0) || (error == 0)) {
        return true;
    }

    return ((prev_error < 0) && (error > 0)) ||
           ((prev_error > 0) && (error < 0));
}

static void APPLOCK_ProcessRampToCenter(void)
{
    /* Move gradually from the scan end point to mid-scale before capture. */
    if (s_lock.output_raw < s_lock.mid_raw) {
        s_lock.output_raw++;
        APPRTLOOP_WriteRaw(s_lock.output_raw);
        s_lock.result.output_raw = s_lock.output_raw;
        return;
    }

    if (s_lock.output_raw > s_lock.mid_raw) {
        s_lock.output_raw--;
        APPRTLOOP_WriteRaw(s_lock.output_raw);
        s_lock.result.output_raw = s_lock.output_raw;
        return;
    }

    APPLOCK_ClearFilter();
    APPLOCK_ClearSlopeVotes();
    s_lock.state = APPLOCK_STATE_SWEEP_DOWN;
}

static void APPLOCK_ProcessSweepDown(uint32_t r_q15)
{
    int32_t error_q15;
    int8_t vote;
    int8_t polarity;

    error_q15 = (int32_t)s_lock.r_target_q15 - (int32_t)r_q15;
    s_lock.result.r_now_q15 = r_q15;
    s_lock.result.error_q15 = error_q15;
    s_lock.result.output_raw = s_lock.output_raw;

    if (s_lock.prev_valid) {
        vote = 0;
        if (r_q15 < s_lock.prev_r_q15) {
            vote = 1;   /* raw is decreasing, so R falling means dR/draw > 0 */
        } else if (r_q15 > s_lock.prev_r_q15) {
            vote = -1;  /* raw is decreasing, so R rising means dR/draw < 0 */
        }

        APPLOCK_AddSlopeVote(vote);

        /* Capture the first crossing, but wait for enough slope votes. */
        if (!s_lock.crossing_seen &&
            APPLOCK_CrossedTarget(s_lock.prev_error_q15, error_q15)) {
            s_lock.crossing_seen = true;
            s_lock.crossing_raw = s_lock.output_raw;
            s_lock.crossing_r_q15 = r_q15;
            s_lock.crossing_error_q15 = error_q15;
        }

        /*
         * Do not fault just because the crossing came before the local slope
         * vote has enough samples. Keep sweeping downward, let the majority
         * vote settle, then return to the captured crossing raw for soft PI.
         */
        if (s_lock.crossing_seen && APPLOCK_DecidePolarity(&polarity)) {
            s_lock.polarity = polarity;
            s_lock.output_raw = s_lock.crossing_raw;
            APPRTLOOP_WriteRaw(s_lock.output_raw);
            APPLOCK_EnterSoft(s_lock.crossing_r_q15, s_lock.crossing_error_q15);
            return;
        }
    }

    s_lock.prev_valid = true;
    s_lock.prev_r_q15 = r_q15;
    s_lock.prev_error_q15 = error_q15;

    if (s_lock.output_raw == 0U) {
        APPLOCK_Fail();
        return;
    }

    s_lock.output_raw--;
    APPRTLOOP_WriteRaw(s_lock.output_raw);
}

static void APPLOCK_EnterSoft(uint32_t r_q15, int32_t error_q15)
{
    s_lock.state = APPLOCK_STATE_SOFT;
    s_lock.integrator_q8 = (int32_t)s_lock.output_raw << 8;

    APPLOCK_ClearFilter();

    s_lock.result.active = true;
    s_lock.result.soft_locked = true;
    s_lock.result.hard_locked = false;
    s_lock.result.error = false;
    s_lock.result.polarity = s_lock.polarity;
    s_lock.result.r_target_q15 = s_lock.r_target_q15;
    s_lock.result.r_now_q15 = r_q15;
    s_lock.result.error_q15 = error_q15;
    s_lock.result.capture_raw = s_lock.output_raw;
    s_lock.result.output_raw = s_lock.output_raw;
    s_lock.result.resonance_done = false;
    s_lock.result.resonance_freq_hz = 0UL;
    s_lock.result.fn_hz = 0UL;
}

static void APPLOCK_ProcessSoft(uint32_t r_q15)
{
    int32_t error_q15;
    int32_t output_q8;

    output_q8 = APPLOCK_RunPiQ8(r_q15, &error_q15);

    s_lock.output_raw = APPLOCK_Q8ToRaw(output_q8);
    APPRTLOOP_WriteRaw(s_lock.output_raw);
}

static int32_t APPLOCK_RunPiQ8(uint32_t r_q15, int32_t *error_out)
{
    int32_t error_q15;
    int32_t control_error;
    int32_t p_q8;

    error_q15 = (int32_t)s_lock.r_target_q15 - (int32_t)r_q15;
    control_error = (int32_t)s_lock.polarity * error_q15;
    control_error = APPLOCK_NormalizeControlErrorQ15(control_error);

    /*
     * Integrator lives in DAC raw Q8 units. The same PI core is used by soft
     * hold and resonance sweep so the injected sine is added on top of the
     * live PI output instead of owning the DAC path separately.
     */
    s_lock.integrator_q8 += (control_error * APPLOCK_SOFT_KI_GAIN_Q8) >> 8;
    s_lock.integrator_q8 = APPLOCK_ClampQ8(s_lock.integrator_q8);

    p_q8 = (control_error * APPLOCK_SOFT_KP_GAIN_Q8) >> 8;

    if (error_out != NULL) {
        *error_out = error_q15;
    }

    return APPLOCK_ClampQ8(s_lock.integrator_q8 + p_q8);
}

static void APPLOCK_ProcessHard(uint32_t r_q15)
{
    int32_t output_q8;

    output_q8 = APPLOCK_RunHardPiQ8(r_q15);
    s_lock.output_raw = APPLOCK_Q8ToRaw(output_q8);
    APPRTLOOP_WriteRaw(s_lock.output_raw);
}

static int32_t APPLOCK_RunHardPiQ8(uint32_t r_q15)
{
    int32_t error_q15;
    int32_t control_error;
    int32_t notched_error;
    int32_t slow_error;
    int32_t p_error;
    int32_t p_q8;

    error_q15 = (int32_t)s_lock.r_target_q15 - (int32_t)r_q15;
    control_error = (int32_t)s_lock.polarity * error_q15;
    control_error = APPLOCK_NormalizeControlErrorQ15(control_error);
    notched_error = APPLOCK_NotchStep(control_error);

#if APPLOCK_HARD_SHELF_LP_SHIFT > 0U
    if (!s_lock.hard_shelf_valid) {
        s_lock.hard_slow_error = notched_error;
        s_lock.hard_shelf_valid = true;
    } else {
        s_lock.hard_slow_error +=
            (notched_error - s_lock.hard_slow_error) >> APPLOCK_HARD_SHELF_LP_SHIFT;
    }
    slow_error = s_lock.hard_slow_error;
#else
    slow_error = notched_error;
#endif

    p_error = slow_error + ((notched_error - slow_error) >> APPLOCK_HARD_SHELF_HF_GAIN_SHIFT);

    s_lock.integrator_q8 += (slow_error * APPLOCK_HARD_KI_GAIN_Q8) >> 8;
    s_lock.integrator_q8 = APPLOCK_ClampQ8(s_lock.integrator_q8);

    p_q8 = (p_error * APPLOCK_HARD_KP_GAIN_Q8) >> 8;
    return APPLOCK_ClampQ8(s_lock.integrator_q8 + p_q8);
}

static int32_t APPLOCK_NormalizeControlErrorQ15(int32_t control_error)
{
    if (s_lock.error_gain_q15 == 0UL) {
        return control_error;
    }

    return (int32_t)(((int64_t)control_error * (int64_t)s_lock.error_gain_q15) >> 15);
}

static void APPLOCK_ConfigNotch(uint32_t freq_hz)
{
    APPLOCK_ConfigNotchStage(&s_lock.notch_main, freq_hz, APPLOCK_NOTCH_BW_HZ);
}

static void APPLOCK_ConfigNotchStage(AppLockNotchState_t *notch,
                                     uint32_t freq_hz,
                                     uint32_t bw_hz)
{
    int32_t cos_q14;
    const int32_t q14_one = (1L << 14);
    const uint32_t quarter_turn_phase = (1UL << 30);
    const uint64_t q14_pi = (((uint64_t)355U << 14) + 56ULL) / 113ULL;
    int32_t width_q14;
    int32_t r_q14;

    /*
     * Approximate width_q14 from bandwidth in hertz:
     *   width_q14 ~= bandwidth_hz * pi * 2^14 / fs
     * This is evaluated only when (re)configuring hard lock, not per sample.
     */
    width_q14 = (int32_t)((((uint64_t)bw_hz * q14_pi) +
                           (APPLOCK_SAMPLE_RATE_HZ / 2U)) / APPLOCK_SAMPLE_RATE_HZ);
    if (width_q14 > q14_one) {
        width_q14 = q14_one;
    }

    r_q14 = q14_one - width_q14;

    cos_q14 = APPLOCK_SineQ14FromPhase(
        (uint32_t)((((uint64_t)freq_hz << 32) / APPLOCK_SAMPLE_RATE_HZ) + quarter_turn_phase));

    notch->b1_q14 = -2L * cos_q14;
    notch->a1_q14 = (int32_t)((2LL * (int64_t)r_q14 * (int64_t)cos_q14) >> 14);
    notch->a2_q14 = -(int32_t)(((int64_t)r_q14 * (int64_t)r_q14) >> 14);
    notch->x1 = 0;
    notch->x2 = 0;
    notch->y1 = 0;
    notch->y2 = 0;
}

static int32_t APPLOCK_NotchStageStep(AppLockNotchState_t *notch, int32_t value)
{
    int64_t acc;
    const int32_t q14_one = (1L << 14);
    int32_t y;

    acc = (int64_t)value * q14_one;
    acc += (int64_t)notch->b1_q14 * (int64_t)notch->x1;
    acc += (int64_t)q14_one * (int64_t)notch->x2;
    acc += (int64_t)notch->a1_q14 * (int64_t)notch->y1;
    acc += (int64_t)notch->a2_q14 * (int64_t)notch->y2;
    y = (int32_t)(acc >> 14);

    notch->x2 = notch->x1;
    notch->x1 = value;
    notch->y2 = notch->y1;
    notch->y1 = y;

    return y;
}

static int32_t APPLOCK_NotchStep(int32_t value)
{
    return APPLOCK_NotchStageStep(&s_lock.notch_main, value);
}

static int32_t APPLOCK_SineQ14FromPhase(uint32_t phase)
{
    uint8_t index;
    uint8_t next_index;
    uint16_t frac;
    int32_t sample;
    int32_t next_sample;

    index = (uint8_t)(phase >> 24);
    next_index = (uint8_t)((index + 1U) & APPLOCK_SINE_TABLE_MASK);
    frac = (uint16_t)((phase >> 8) & 0xFFFFU);
    sample = s_lock_sine_q15[index];
    next_sample = s_lock_sine_q15[next_index];
    sample += (int32_t)(((int64_t)(next_sample - sample) * (int64_t)frac) >> 16);

    return sample / 2;
}

static void APPLOCK_StartResonancePoint(uint32_t freq_hz,
                                        uint32_t step_hz,
                                        uint32_t end_hz,
                                        uint16_t samples_target,
                                        AppLockResonanceStage_t stage)
{
    s_lock.resonance_stage = stage;
    s_lock.resonance_freq_hz = freq_hz;
    s_lock.resonance_step_hz = step_hz;
    s_lock.resonance_end_hz = end_hz;
    s_lock.resonance_samples_target = samples_target;
    s_lock.resonance_samples_done = 0U;
    s_lock.resonance_phase = 0UL;
    s_lock.resonance_phase_step =
        (uint32_t)(((uint64_t)freq_hz << 32) / APPLOCK_SAMPLE_RATE_HZ);
    s_lock.resonance_i_acc = 0;
    s_lock.resonance_q_acc = 0;

    s_lock.result.resonance_freq_hz = freq_hz;
}

static void APPLOCK_ProcessResonance(uint32_t r_q15)
{
    uint8_t sin_index;
    uint8_t cos_index;
    int16_t sin_q15;
    int16_t cos_q15;
    int32_t error_q15;
    int32_t pi_output_q8;
    int32_t inj_raw;
    int32_t output_q8;

    pi_output_q8 = APPLOCK_RunPiQ8(r_q15, &error_q15);

    /*
     * DDS injection and IQ references share the same phase accumulator. The
     * sine perturbs the PZT, and the cosine/sine pair measures the response
     * at exactly that injected frequency.
     */
    s_lock.resonance_phase += s_lock.resonance_phase_step;
    sin_index = (uint8_t)(s_lock.resonance_phase >> 24);
    cos_index = (uint8_t)((sin_index + APPLOCK_COS_TABLE_OFFSET) & APPLOCK_SINE_TABLE_MASK);
    sin_q15 = s_lock_sine_q15[sin_index];
    cos_q15 = s_lock_sine_q15[cos_index];

    inj_raw = ((int32_t)APPLOCK_RESONANCE_INJ_AMP_RAW * (int32_t)sin_q15) >> 15;
    output_q8 = APPLOCK_ClampQ8(pi_output_q8 + (inj_raw << 8));
    s_lock.output_raw = APPLOCK_Q8ToRaw(output_q8);
    APPRTLOOP_WriteRaw(s_lock.output_raw);

    s_lock.resonance_i_acc += (int64_t)error_q15 * (int64_t)cos_q15;
    s_lock.resonance_q_acc += (int64_t)error_q15 * (int64_t)sin_q15;
    s_lock.resonance_samples_done++;

    if (s_lock.resonance_samples_done >= s_lock.resonance_samples_target) {
        APPLOCK_FinishResonancePoint();
    }
}

static void APPLOCK_FinishResonancePoint(void)
{
    uint64_t amp2;
    uint32_t next_freq;
    uint32_t fine_start;
    uint32_t fine_end;

    amp2 = APPLOCK_ComputeIqAmp2();
    if ((s_lock.resonance_best_hz == 0UL) || (amp2 > s_lock.resonance_best_amp2)) {
        s_lock.resonance_best_amp2 = amp2;
        s_lock.resonance_best_hz = s_lock.resonance_freq_hz;
    }

    next_freq = s_lock.resonance_freq_hz + s_lock.resonance_step_hz;
    if (next_freq <= s_lock.resonance_end_hz) {
        APPLOCK_StartResonancePoint(next_freq,
                                    s_lock.resonance_step_hz,
                                    s_lock.resonance_end_hz,
                                    s_lock.resonance_samples_target,
                                    s_lock.resonance_stage);
        return;
    }

    if (s_lock.resonance_stage == APPLOCK_RESONANCE_STAGE_COARSE) {
        fine_start = (s_lock.resonance_best_hz > APPLOCK_RESONANCE_FINE_SPAN_HZ) ?
                     (s_lock.resonance_best_hz - APPLOCK_RESONANCE_FINE_SPAN_HZ) :
                     APPLOCK_RESONANCE_COARSE_START_HZ;
        if (fine_start < APPLOCK_RESONANCE_COARSE_START_HZ) {
            fine_start = APPLOCK_RESONANCE_COARSE_START_HZ;
        }

        fine_end = s_lock.resonance_best_hz + APPLOCK_RESONANCE_FINE_SPAN_HZ;
        if (fine_end > APPLOCK_RESONANCE_COARSE_END_HZ) {
            fine_end = APPLOCK_RESONANCE_COARSE_END_HZ;
        }

        s_lock.resonance_best_amp2 = 0ULL;
        s_lock.resonance_best_hz = 0UL;
        APPLOCK_StartResonancePoint(fine_start,
                                    APPLOCK_RESONANCE_FINE_STEP_HZ,
                                    fine_end,
                                    APPLOCK_RESONANCE_FINE_SAMPLES,
                                    APPLOCK_RESONANCE_STAGE_FINE);
        return;
    }

    /*
     * TODO: estimate Q from half-power bandwidth after storing or streaming
     * per-frequency response values. First version only reports fn.
     */
    s_lock.state = APPLOCK_STATE_SOFT;
    s_lock.resonance_stage = APPLOCK_RESONANCE_STAGE_IDLE;
    s_lock.result.hard_locked = false;
    s_lock.result.resonance_done = true;
    s_lock.result.fn_hz = s_lock.resonance_best_hz;
    s_lock.result.resonance_freq_hz = s_lock.resonance_best_hz;
    APPLOCK_ClearFilter();
}

static uint64_t APPLOCK_ComputeIqAmp2(void)
{
    int64_t i_scaled;
    int64_t q_scaled;
    uint64_t i_abs;
    uint64_t q_abs;

    /*
     * Compare I^2 + Q^2 only at frequency boundaries. The shift keeps the
     * square inside uint64_t while preserving enough relative peak information
     * for first-cut resonance selection.
     */
    i_scaled = s_lock.resonance_i_acc >> APPLOCK_IQ_AMP_SHIFT;
    q_scaled = s_lock.resonance_q_acc >> APPLOCK_IQ_AMP_SHIFT;
    i_abs = (i_scaled < 0) ? (uint64_t)(-i_scaled) : (uint64_t)i_scaled;
    q_abs = (q_scaled < 0) ? (uint64_t)(-q_scaled) : (uint64_t)q_scaled;

    return (i_abs * i_abs) + (q_abs * q_abs);
}

static void APPLOCK_Fail(void)
{
    s_lock.error = true;
    s_lock.active = false;
    s_lock.state = APPLOCK_STATE_IDLE;
    s_lock.result.error = true;
    s_lock.result.active = false;
    s_lock.result.soft_locked = false;
    s_lock.result.hard_locked = false;
    APPRTLOOP_Stop();
}

static int32_t APPLOCK_ClampQ8(int32_t value_q8)
{
    int32_t max_q8 = (int32_t)s_lock.max_raw << 8;

    if (value_q8 < 0) {
        return 0;
    }

    if (value_q8 > max_q8) {
        return max_q8;
    }

    return value_q8;
}

static uint16_t APPLOCK_Q8ToRaw(int32_t value_q8)
{
    return (uint16_t)(APPLOCK_ClampQ8(value_q8) >> 8);
}
