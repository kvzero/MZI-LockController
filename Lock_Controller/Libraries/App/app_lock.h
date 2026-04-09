#ifndef APP_LOCK_H
#define APP_LOCK_H

#include <stdbool.h>
#include <stdint.h>
#include "app_context.h"
#include "app_rtloop.h"

/**
 * @brief Clear the lock backend and stop the realtime loop if lock owns it.
 */
void APPLOCK_Reset(void);

/**
 * @brief Start low-bandwidth soft-lock capture using the current R target.
 *
 * The backend ramps the PZT output to the middle of the configured HVDAC range,
 * sweeps down to find the R-target crossing, derives the local slope polarity,
 * then enters conservative PI hold.
 */
bool APPLOCK_StartSoft(uint16_t iout_offset_raw,
                       uint16_t iref_offset_raw,
                       const AppScanResult_t *scan_result);

/**
 * @brief Stop the active lock path, if any.
 */
void APPLOCK_Stop(void);

/**
 * @brief Start the first-cut resonance sweep while soft PI keeps holding lock.
 *
 * The sweep adds a small DDS sine injection on top of the PI output and uses
 * IQ accumulation to find the strongest response frequency. Q estimation and
 * notch configuration are intentionally left for later steps.
 */
bool APPLOCK_StartResonanceSweep(void);

/**
 * @brief Switch from soft hold into hard lock using the measured resonance fn.
 *
 * The hard path keeps the same realtime loop active, enables a fixed-point
 * notch around the measured resonance, and applies the harder PI gains.
 */
bool APPLOCK_StartHardLock(void);

/**
 * @brief Check whether soft-lock capture or hold has failed.
 */
bool APPLOCK_HasError(void);

/**
 * @brief Read the latest public lock state for UI/process decisions.
 */
const AppLockRuntime_t *APPLOCK_GetResult(void);

/**
 * @brief Consume one post-offset sample pair from the realtime loop.
 *
 * @note Call this only from APPRTLOOP's sample callback path.
 */
void APPLOCK_OnSample(const AppRtloopSample_t *sample);

#endif /* APP_LOCK_H */
