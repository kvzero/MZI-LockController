#ifndef APP_SCAN_H
#define APP_SCAN_H

#include <stdbool.h>
#include <stdint.h>

#include "app_rtloop.h"

typedef struct {
    uint8_t  cycles_done;
    uint8_t  cycles_total;
    uint16_t contrast_q15;
    uint32_t r_max_q15;
    uint32_t r_min_q15;
    uint32_t r_target_q15;
    bool     valid;
} AppScanResult_t;

/**
 * @brief Reset all scan state back to idle and clear the last result.
 */
void APPSCAN_Reset(void);

/**
 * @brief Start a full-range scan using the shared sample-by-sample loop path.
 *
 * @param cycles_total Number of full triangle-wave cycles to execute.
 * @param iout_offset_raw Dark/background offset for the output PD channel.
 * @param iref_offset_raw Dark/background offset for the reference PD channel.
 * @retval true Scan armed successfully.
 * @retval false Scan is already active or the realtime loop could not start.
 */
bool APPSCAN_Start(uint8_t cycles_total,
                   uint16_t iout_offset_raw,
                   uint16_t iref_offset_raw);

/**
 * @brief Stop the active scan, if any.
 */
void APPSCAN_Stop(void);

/**
 * @brief Check whether a scan is currently active.
 */
bool APPSCAN_IsActive(void);

/**
 * @brief Check whether the current scan has finished and published a result.
 */
bool APPSCAN_IsDone(void);

/**
 * @brief Read the most recent scan result summary.
 */
const AppScanResult_t *APPSCAN_GetResult(void);

/**
 * @brief Check whether the scan stopped because post-offset Iref fell below the shared threshold.
 */
bool APPSCAN_HasRefLow(void);

/**
 * @brief Consume one post-offset sample pair from the shared realtime loop.
 *
 * @note Call this only from APPRTLOOP's realtime callback path.
 */
void APPSCAN_OnSample(const AppRtloopSample_t *sample);

#endif /* APP_SCAN_H */
