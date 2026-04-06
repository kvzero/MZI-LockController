#ifndef APP_RTLOOP_H
#define APP_RTLOOP_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Runtime mode currently owning the 250 kHz sample-by-sample path.
 */
typedef enum {
    APP_RTLOOP_MODE_IDLE = 0,
    APP_RTLOOP_MODE_SCAN,
    APP_RTLOOP_MODE_LOCK,
} AppRtloopMode_t;

/**
 * @brief One dual-PD sample pair prepared for app-level realtime processing.
 *
 * @note `iout` and `iref` are the post-offset signed values derived from the
 *       raw ADC sample pair. They are not filtered or ratio-normalized here.
 */
typedef struct {
    uint16_t iout_raw;
    uint16_t iref_raw;
    int32_t  iout;
    int32_t  iref;
} AppRtloopSample_t;

/**
 * @brief Reset the realtime loop state back to idle.
 */
void APPRTLOOP_Init(void);

/**
 * @brief Start the shared realtime path in scan mode.
 *
 * @param iout_offset_raw Dark/background offset for the output PD channel.
 * @param iref_offset_raw Dark/background offset for the reference PD channel.
 * @retval true Sample-by-sample DMA streaming started successfully.
 * @retval false Hardware path is unavailable or already active.
 */
bool APPRTLOOP_StartScan(uint16_t iout_offset_raw, uint16_t iref_offset_raw);

/**
 * @brief Start the shared realtime path in lock mode.
 *
 * @param iout_offset_raw Dark/background offset for the output PD channel.
 * @param iref_offset_raw Dark/background offset for the reference PD channel.
 * @retval true Sample-by-sample DMA streaming started successfully.
 * @retval false Hardware path is unavailable or already active.
 */
bool APPRTLOOP_StartLock(uint16_t iout_offset_raw, uint16_t iref_offset_raw);

/**
 * @brief Stop the active realtime path, if any.
 */
void APPRTLOOP_Stop(void);

/**
 * @brief Check whether the realtime loop currently owns the PDADC stream.
 */
bool APPRTLOOP_IsActive(void);

/**
 * @brief Check whether the last active stream terminated because of ADC/DMA error.
 */
bool APPRTLOOP_HasError(void);

/**
 * @brief Read the currently armed realtime mode.
 */
AppRtloopMode_t APPRTLOOP_GetMode(void);

/**
 * @brief Write one raw DAC code through the shared loop output path.
 *
 * @note This helper keeps track of the last raw code written by app-level
 *       realtime logic. Hardware clamping still happens inside HVDAC_WriteRaw().
 */
void APPRTLOOP_WriteRaw(uint16_t raw);

/**
 * @brief Read the last raw DAC code issued by APPRTLOOP_WriteRaw().
 */
uint16_t APPRTLOOP_GetLastRaw(void);

#endif /* APP_RTLOOP_H */
