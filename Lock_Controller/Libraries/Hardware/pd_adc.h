#ifndef PD_ADC_H
#define PD_ADC_H

#include <stdbool.h>
#include <stdint.h>

#include "adc.h"
#include "tim.h"

/**
 * @brief One synchronized dual-ADC sample pair unpacked from the common CDR register.
 */
typedef struct {
    uint16_t pd_a;
    uint16_t pd_b;
} PDADC_Frame_t;

/**
 * @brief Called from DMA IRQ context when one complete batch of sample pairs is ready.
 *
 * @param cdr_buf Pointer to packed CDR words. Each word contains one synchronized sample pair.
 * @param frame_count Number of sample pairs in this batch.
 * @param user_ctx User context passed to the DMA-based start API.
 *
 * @note For `PDADC_StartOneshot()`, the buffer is already frozen when this callback runs.
 *       For `PDADC_StartContinuous()`, the stream is still active when this callback runs.
 */
typedef void (*PDADC_StreamDoneCallback_t)(const uint32_t *cdr_buf,
                                           uint16_t frame_count,
                                           void *user_ctx);

/**
 * @brief Called from IRQ context when the active stream aborts because DMA or ADC reported an error.
 *
 * @param user_ctx User context passed to the DMA-based start API.
 */
typedef void (*PDADC_StreamErrorCallback_t)(void *user_ctx);

typedef enum {
    PDADC_MODE_NONE = 0,
    PDADC_MODE_ONESHOT,
    PDADC_MODE_CONTINUOUS,
} PDADC_Mode_t;

/**
 * @brief Dual-ADC acquisition handle for blocking reads and DMA-based acquisition APIs.
 *
 * @note `htim_trig` is used only by DMA-based acquisition APIs. PDADC_Read() always uses a software trigger.
 */
typedef struct {
    ADC_HandleTypeDef *hadc_master;
    ADC_HandleTypeDef *hadc_slave;
    TIM_HandleTypeDef *htim_trig;
    volatile uint8_t   busy;
    PDADC_Mode_t       mode;

    uint32_t                    *stream_buf;
    uint16_t                     stream_frame_count;
    PDADC_StreamDoneCallback_t   stream_done_cb;
    PDADC_StreamErrorCallback_t  stream_error_cb;
    void                        *stream_user_ctx;
} PDADC_Handle_t;

/**
 * @brief Unpack one common-data-register word into a synchronized ADC sample pair.
 *
 * @param cdr_word Packed CDR value. Low 16 bits are master ADC, high 16 bits are slave ADC.
 * @return Unpacked sample pair.
 */
static inline PDADC_Frame_t PDADC_UnpackCDR(uint32_t cdr_word)
{
    PDADC_Frame_t frame;

    frame.pd_a = (uint16_t)(cdr_word & 0xFFFFU);
    frame.pd_b = (uint16_t)((cdr_word >> 16) & 0xFFFFU);
    return frame;
}

/**
 * @brief Prepare the dual-ADC handle for use and run single-ended calibration on both ADCs.
 *
 * @note Call this once after `MX_ADC1_Init()`, `MX_ADC2_Init()`, and `MX_TIM1_Init()`.
 *       The trigger timer is forced idle and any stale software stream state is cleared.
 *
 * @param h Acquisition handle.
 */
void PDADC_Init(PDADC_Handle_t *h);

/**
 * @brief Read one synchronized sample pair using a software-triggered single conversion.
 *
 * @note This is a blocking low-rate path intended for offset acquisition and debug reads.
 *       It does not start TIM1, DMA, or any interrupt-driven path.
 *
 * @param h Acquisition handle.
 * @param out Destination for the synchronized sample pair.
 * @retval true Read completed successfully.
 * @retval false Handle is invalid, ADC was busy, or the conversion timed out.
 */
bool PDADC_Read(PDADC_Handle_t *h, PDADC_Frame_t *out);

/**
 * @brief Start one timer-triggered DMA batch and stop automatically after it is filled.
 *
 * @param h Acquisition handle.
 * @param cdr_buf Destination buffer for packed CDR words.
 * @param frame_count Number of synchronized sample pairs to capture.
 * @param done_cb Completion callback for the captured batch. Must not be NULL.
 * @param error_cb Error callback for DMA/ADC faults. May be NULL.
 * @param user_ctx User context forwarded to callbacks.
 * @retval true Capture armed successfully and is waiting for TIM trigger events.
 * @retval false Handle is invalid, a capture is already active, or DMA start failed.
 */
bool PDADC_StartOneshot(PDADC_Handle_t *h,
                        uint32_t *cdr_buf,
                        uint16_t frame_count,
                        PDADC_StreamDoneCallback_t done_cb,
                        PDADC_StreamErrorCallback_t error_cb,
                        void *user_ctx);

/**
 * @brief Start timer-triggered DMA streaming in continuous mode.
 *
 * @param h Acquisition handle.
 * @param cdr_buf Destination buffer for packed CDR words.
 * @param frame_count Number of synchronized sample pairs per callback.
 * @param done_cb Completion callback for one full batch. Must not be NULL.
 * @param error_cb Error callback for DMA/ADC faults. May be NULL.
 * @param user_ctx User context forwarded to callbacks.
 * @retval true Stream armed successfully and is waiting for TIM trigger events.
 * @retval false Handle is invalid, a stream is already active, or DMA start failed.
 */
bool PDADC_StartContinuous(PDADC_Handle_t *h,
                           uint32_t *cdr_buf,
                           uint16_t frame_count,
                           PDADC_StreamDoneCallback_t done_cb,
                           PDADC_StreamErrorCallback_t error_cb,
                           void *user_ctx);

/**
 * @brief Stop the active continuous DMA stream for this handle.
 *
 * @param h Acquisition handle.
 */
void PDADC_StopContinuous(PDADC_Handle_t *h);

/**
 * @brief Handle DMA transfer-complete and DMA error events for the active PDADC stream.
 *
 * @note Call this only from `DMA1_Channel1_IRQHandler()`.
 */
void PDADC_DMA_IRQHandler(void);

#endif /* PD_ADC_H */
