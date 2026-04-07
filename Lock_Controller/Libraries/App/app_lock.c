#include "app_lock.h"

#include "app_hardware.h"

/*
 * Soft-lock capture path:
 *   1. Ramp the PZT output to the HVDAC mid raw code to avoid a voltage step.
 *   2. Sweep downward one raw code per realtime sample until R crosses Rtarget.
 *   3. Use a short majority vote on the local slope to choose PI polarity.
 *   4. Enter a conservative low-bandwidth PI hold.
 *
 * Resonance identification and hard-lock mode will reuse this backend later,
 * but they are intentionally not implemented in this first soft-lock step.
 */
#define APPLOCK_MEDIAN_WINDOW       7U
#define APPLOCK_SLOPE_VOTE_WINDOW  32U
#define APPLOCK_SLOPE_MIN_VOTES     8U

/*
 * First-cut soft-lock gains. The integrator is stored in DAC raw Q8 units so
 * the controller can make sub-raw-code accumulated moves while still writing
 * integer raw codes to the DAC. These constants are intentionally local and
 * must be tuned on the real interferometer.
 */
#define APPLOCK_SOFT_KP_Q8_SHIFT    2U
#define APPLOCK_SOFT_KI_Q8_SHIFT    7U

typedef enum {
    APPLOCK_STATE_IDLE = 0,
    APPLOCK_STATE_RAMP_TO_CENTER,
    APPLOCK_STATE_SWEEP_DOWN,
    APPLOCK_STATE_SOFT,
} AppLockState_t;

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
static void APPLOCK_Fail(void);
static int32_t APPLOCK_ClampQ8(int32_t value_q8);
static uint16_t APPLOCK_Q8ToRaw(int32_t value_q8);

void APPLOCK_Reset(void)
{
    if (s_lock.active) {
        APPRTLOOP_Stop();
    }

    APPLOCK_ClearState();
}

bool APPLOCK_StartSoft(uint16_t iout_offset_raw,
                       uint16_t iref_offset_raw,
                       uint32_t r_target_q15)
{
    if (s_lock.active ||
        (g_hw == NULL) ||
        (g_hw->hvdac == NULL) ||
        (g_hw->hpdadc == NULL) ||
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
    s_lock.r_target_q15 = r_target_q15;

    s_lock.result.active = true;
    s_lock.result.r_target_q15 = r_target_q15;
    s_lock.result.output_raw = s_lock.output_raw;

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

    for (i = 0U; i < APPLOCK_MEDIAN_WINDOW; ++i) {
        s_lock.median_buf[i] = 0UL;
    }

    for (i = 0U; i < APPLOCK_SLOPE_VOTE_WINDOW; ++i) {
        s_lock.slope_votes[i] = 0;
    }

    s_lock.result.active = false;
    s_lock.result.soft_locked = false;
    s_lock.result.error = false;
    s_lock.result.polarity = 0;
    s_lock.result.r_target_q15 = 0U;
    s_lock.result.r_now_q15 = 0U;
    s_lock.result.error_q15 = 0;
    s_lock.result.capture_raw = 0U;
    s_lock.result.output_raw = 0U;
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
    s_lock.result.error = false;
    s_lock.result.polarity = s_lock.polarity;
    s_lock.result.r_target_q15 = s_lock.r_target_q15;
    s_lock.result.r_now_q15 = r_q15;
    s_lock.result.error_q15 = error_q15;
    s_lock.result.capture_raw = s_lock.output_raw;
    s_lock.result.output_raw = s_lock.output_raw;
}

static void APPLOCK_ProcessSoft(uint32_t r_q15)
{
    int32_t error_q15;
    int32_t control_error;
    int32_t p_q8;
    int32_t output_q8;

    error_q15 = (int32_t)s_lock.r_target_q15 - (int32_t)r_q15;
    control_error = (int32_t)s_lock.polarity * error_q15;

    /* Integrator lives in DAC raw Q8 units and is clamped to the safe range. */
    s_lock.integrator_q8 += control_error / (int32_t)(1UL << APPLOCK_SOFT_KI_Q8_SHIFT);
    s_lock.integrator_q8 = APPLOCK_ClampQ8(s_lock.integrator_q8);

    p_q8 = control_error / (int32_t)(1UL << APPLOCK_SOFT_KP_Q8_SHIFT);
    output_q8 = APPLOCK_ClampQ8(s_lock.integrator_q8 + p_q8);

    s_lock.output_raw = APPLOCK_Q8ToRaw(output_q8);
    APPRTLOOP_WriteRaw(s_lock.output_raw);
}

static void APPLOCK_Fail(void)
{
    s_lock.error = true;
    s_lock.active = false;
    s_lock.state = APPLOCK_STATE_IDLE;
    s_lock.result.error = true;
    s_lock.result.active = false;
    s_lock.result.soft_locked = false;
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
