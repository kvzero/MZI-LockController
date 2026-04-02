#include "pd_adc.h"

static PDADC_Handle_t *g_pdadc_active = NULL;

static bool PDADC_IsHandleValid(const PDADC_Handle_t *h);
static bool PDADC_EnableADC(ADC_HandleTypeDef *hadc);
static bool PDADC_StartStream(PDADC_Handle_t *h,
                              uint32_t *cdr_buf,
                              uint16_t frame_count,
                              PDADC_StreamDoneCallback_t done_cb,
                              PDADC_StreamErrorCallback_t error_cb,
                              void *user_ctx);
static void PDADC_ResetStreamState(PDADC_Handle_t *h);
static void PDADC_StopActiveStream(bool notify_error);

void PDADC_Init(PDADC_Handle_t *h)
{
    if (!PDADC_IsHandleValid(h) || (h->busy != 0U) || (g_pdadc_active != NULL)) {
        Error_Handler();
    }

    if (h->htim_trig != NULL) {
        (void)HAL_TIM_Base_Stop(h->htim_trig);
        __HAL_TIM_SET_COUNTER(h->htim_trig, 0U);
    }

    PDADC_ResetStreamState(h);
    __HAL_ADC_DISABLE_IT(h->hadc_master, ADC_IT_OVR);
    __HAL_ADC_DISABLE_IT(h->hadc_slave, ADC_IT_OVR);
    __HAL_ADC_CLEAR_FLAG(h->hadc_master, ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_EOS);
    __HAL_ADC_CLEAR_FLAG(h->hadc_slave, ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_EOS);

    if (HAL_ADCEx_Calibration_Start(h->hadc_slave, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_ADCEx_Calibration_Start(h->hadc_master, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
}

bool PDADC_Read(PDADC_Handle_t *h, PDADC_Frame_t *out)
{
    ADC_Common_TypeDef *adc_common = NULL;
    uint32_t saved_master_exten = 0U;
    uint32_t saved_common_ccr = 0U;
    uint32_t tickstart = 0U;
    bool ok = false;

    if (!PDADC_IsHandleValid(h) || (out == NULL) || (h->busy != 0U) || (g_pdadc_active != NULL)) {
        return false;
    }

    h->busy = 1U;
    adc_common = __LL_ADC_COMMON_INSTANCE(h->hadc_master->Instance);
    saved_master_exten = READ_BIT(h->hadc_master->Instance->CFGR, ADC_CFGR_EXTEN);
    saved_common_ccr = READ_REG(adc_common->CCR);

    CLEAR_BIT(h->hadc_master->Instance->CFGR, ADC_CFGR_EXTEN);
    CLEAR_BIT(adc_common->CCR, ADC_CCR_MDMA | ADC_CCR_DMACFG);

    if (!PDADC_EnableADC(h->hadc_slave) || !PDADC_EnableADC(h->hadc_master)) {
        goto done;
    }

    __HAL_ADC_CLEAR_FLAG(h->hadc_master, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(h->hadc_slave, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);

    LL_ADC_REG_StartConversion(h->hadc_master->Instance);

    tickstart = HAL_GetTick();
    while (__HAL_ADC_GET_FLAG(h->hadc_master, ADC_FLAG_EOS) == 0U) {
        if ((HAL_GetTick() - tickstart) > 10U) {
            goto done;
        }
    }

    *out = PDADC_UnpackCDR(HAL_ADCEx_MultiModeGetValue(h->hadc_master));

    __HAL_ADC_CLEAR_FLAG(h->hadc_master, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
    __HAL_ADC_CLEAR_FLAG(h->hadc_slave, ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR);
    ok = true;

done:
    MODIFY_REG(h->hadc_master->Instance->CFGR, ADC_CFGR_EXTEN, saved_master_exten);
    MODIFY_REG(adc_common->CCR, ADC_CCR_MDMA | ADC_CCR_DMACFG, saved_common_ccr);
    h->busy = 0U;
    return ok;
}

bool PDADC_StartOneshot(PDADC_Handle_t *h,
                        uint32_t *cdr_buf,
                        uint16_t frame_count,
                        PDADC_StreamDoneCallback_t done_cb,
                        PDADC_StreamErrorCallback_t error_cb,
                        void *user_ctx)
{
    DMA_HandleTypeDef *hdma = NULL;

    if (!PDADC_IsHandleValid(h) || (h->hadc_master->DMA_Handle == NULL)) {
        return false;
    }

    hdma = h->hadc_master->DMA_Handle;
    h->mode = PDADC_MODE_ONESHOT;
    hdma->Init.Mode = DMA_NORMAL;
    CLEAR_BIT(hdma->Instance->CCR, DMA_CCR_CIRC);

    if (!PDADC_StartStream(h, cdr_buf, frame_count, done_cb, error_cb, user_ctx)) {
        h->mode = PDADC_MODE_NONE;
        return false;
    }

    return true;
}

bool PDADC_StartContinuous(PDADC_Handle_t *h,
                           uint32_t *cdr_buf,
                           uint16_t frame_count,
                           PDADC_StreamDoneCallback_t done_cb,
                           PDADC_StreamErrorCallback_t error_cb,
                           void *user_ctx)
{
    DMA_HandleTypeDef *hdma = NULL;

    if (!PDADC_IsHandleValid(h) || (h->hadc_master->DMA_Handle == NULL)) {
        return false;
    }

    hdma = h->hadc_master->DMA_Handle;
    h->mode = PDADC_MODE_CONTINUOUS;
    hdma->Init.Mode = DMA_CIRCULAR;
    SET_BIT(hdma->Instance->CCR, DMA_CCR_CIRC);

    if (!PDADC_StartStream(h, cdr_buf, frame_count, done_cb, error_cb, user_ctx)) {
        h->mode = PDADC_MODE_NONE;
        return false;
    }

    return true;
}

static bool PDADC_StartStream(PDADC_Handle_t *h,
                              uint32_t *cdr_buf,
                              uint16_t frame_count,
                              PDADC_StreamDoneCallback_t done_cb,
                              PDADC_StreamErrorCallback_t error_cb,
                              void *user_ctx)
{
    DMA_HandleTypeDef *hdma = NULL;

    if (!PDADC_IsHandleValid(h) ||
        (h->htim_trig == NULL) ||
        (cdr_buf == NULL) ||
        (frame_count == 0U) ||
        (done_cb == NULL) ||
        (h->busy != 0U) ||
        (g_pdadc_active != NULL) ||
        (h->mode == PDADC_MODE_NONE) ||
        (h->hadc_master->DMA_Handle == NULL)) {
        return false;
    }

    h->stream_buf = cdr_buf;
    h->stream_frame_count = frame_count;
    h->stream_done_cb = done_cb;
    h->stream_error_cb = error_cb;
    h->stream_user_ctx = user_ctx;
    h->busy = 1U;
    g_pdadc_active = h;

    if (HAL_ADCEx_MultiModeStart_DMA(h->hadc_master, cdr_buf, frame_count) != HAL_OK) {
        PDADC_ResetStreamState(h);
        return false;
    }

    hdma = h->hadc_master->DMA_Handle;
    hdma->XferCpltCallback = NULL;
    hdma->XferHalfCpltCallback = NULL;
    hdma->XferErrorCallback = NULL;
    __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT);

    __HAL_TIM_SET_COUNTER(h->htim_trig, 0U);
    if (HAL_TIM_Base_Start(h->htim_trig) != HAL_OK) {
        PDADC_StopActiveStream(false);
        return false;
    }

    return true;
}

void PDADC_StopContinuous(PDADC_Handle_t *h)
{
    if (!PDADC_IsHandleValid(h) || (g_pdadc_active != h) || (h->mode != PDADC_MODE_CONTINUOUS)) {
        return;
    }

    PDADC_StopActiveStream(false);
}

void PDADC_DMA_IRQHandler(void)
{
    PDADC_Handle_t *h = g_pdadc_active;
    DMA_HandleTypeDef *hdma;
    PDADC_StreamDoneCallback_t done_cb;
    uint32_t *stream_buf;
    uint16_t stream_frame_count;
    void *stream_user_ctx;
    uint32_t flag_it;
    uint32_t source_it;
    uint32_t channel_shift;

    if (!PDADC_IsHandleValid(h) || (h->hadc_master->DMA_Handle == NULL)) {
        CLEAR_BIT(DMA1_Channel1->CCR, DMA_CCR_EN | DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE);
        DMA1->IFCR = DMA_IFCR_CGIF1;
        return;
    }

    hdma = h->hadc_master->DMA_Handle;
    flag_it = hdma->DmaBaseAddress->ISR;
    source_it = hdma->Instance->CCR;
    channel_shift = (hdma->ChannelIndex & 0x1FU);

    if (((flag_it & ((uint32_t)DMA_FLAG_TE1 << channel_shift)) != 0U) &&
        ((source_it & DMA_IT_TE) != 0U)) {
        hdma->DmaBaseAddress->IFCR = ((uint32_t)DMA_ISR_GIF1 << channel_shift);
        hdma->ErrorCode = HAL_DMA_ERROR_TE;
        hdma->State = HAL_DMA_STATE_READY;
        __HAL_UNLOCK(hdma);
        PDADC_StopActiveStream(true);
        return;
    }

    if (((flag_it & ((uint32_t)DMA_FLAG_TC1 << channel_shift)) != 0U) &&
        ((source_it & DMA_IT_TC) != 0U)) {
        hdma->DmaBaseAddress->IFCR = ((uint32_t)DMA_ISR_TCIF1 << channel_shift);
        __HAL_UNLOCK(hdma);

        if (h->mode == PDADC_MODE_ONESHOT) {
            done_cb = h->stream_done_cb;
            stream_buf = h->stream_buf;
            stream_frame_count = h->stream_frame_count;
            stream_user_ctx = h->stream_user_ctx;

            PDADC_StopActiveStream(false);

            if (done_cb != NULL) {
                done_cb(stream_buf, stream_frame_count, stream_user_ctx);
            }
            return;
        }

        if (h->stream_done_cb != NULL) {
            h->stream_done_cb(h->stream_buf, h->stream_frame_count, h->stream_user_ctx);
        }
        return;
    }

}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    PDADC_Handle_t *h = g_pdadc_active;

    if (!PDADC_IsHandleValid(h)) {
        return;
    }

    if ((hadc != h->hadc_master) && (hadc != h->hadc_slave)) {
        return;
    }

    PDADC_StopActiveStream(true);
}

static bool PDADC_IsHandleValid(const PDADC_Handle_t *h)
{
    return (h != NULL) &&
           (h->hadc_master != NULL) &&
           (h->hadc_slave != NULL);
}

static bool PDADC_EnableADC(ADC_HandleTypeDef *hadc)
{
    uint32_t tickstart = 0U;

    if (LL_ADC_IsEnabled(hadc->Instance) != 0U) {
        return true;
    }

    if ((hadc->Instance->CR & (ADC_CR_ADCAL | ADC_CR_JADSTP | ADC_CR_ADSTP |
                               ADC_CR_JADSTART | ADC_CR_ADSTART | ADC_CR_ADDIS | ADC_CR_ADEN)) != 0U) {
        return false;
    }

    LL_ADC_Enable(hadc->Instance);
    tickstart = HAL_GetTick();

    while (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_RDY) == 0U) {
        if (LL_ADC_IsEnabled(hadc->Instance) == 0U) {
            LL_ADC_Enable(hadc->Instance);
        }

        if ((HAL_GetTick() - tickstart) > 2U) {
            if (__HAL_ADC_GET_FLAG(hadc, ADC_FLAG_RDY) == 0U) {
                return false;
            }
        }
    }

    return true;
}

static void PDADC_ResetStreamState(PDADC_Handle_t *h)
{
    if (h == NULL) {
        return;
    }

    h->mode = PDADC_MODE_NONE;
    h->stream_buf = NULL;
    h->stream_frame_count = 0U;
    h->stream_done_cb = NULL;
    h->stream_error_cb = NULL;
    h->stream_user_ctx = NULL;
    h->busy = 0U;

    if (g_pdadc_active == h) {
        g_pdadc_active = NULL;
    }
}

static void PDADC_StopActiveStream(bool notify_error)
{
    PDADC_Handle_t *h = g_pdadc_active;
    DMA_HandleTypeDef *hdma;
    PDADC_StreamErrorCallback_t error_cb;
    void *user_ctx;

    if (!PDADC_IsHandleValid(h)) {
        return;
    }

    error_cb = h->stream_error_cb;
    user_ctx = h->stream_user_ctx;
    hdma = h->hadc_master->DMA_Handle;

    if (h->htim_trig != NULL) {
        __HAL_TIM_DISABLE(h->htim_trig);
        __HAL_TIM_SET_COUNTER(h->htim_trig, 0U);
    }

    __HAL_ADC_DISABLE_IT(h->hadc_master, ADC_IT_OVR);
    __HAL_ADC_CLEAR_FLAG(h->hadc_master, ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_EOS);
    __HAL_ADC_CLEAR_FLAG(h->hadc_slave, ADC_FLAG_OVR | ADC_FLAG_EOC | ADC_FLAG_EOS);

    if (hdma != NULL) {
        __HAL_DMA_DISABLE_IT(hdma, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE);
        __HAL_DMA_DISABLE(hdma);
        hdma->DmaBaseAddress->IFCR = ((uint32_t)DMA_ISR_GIF1 << (hdma->ChannelIndex & 0x1FU));
        hdma->State = HAL_DMA_STATE_READY;
        __HAL_UNLOCK(hdma);
    }

    CLEAR_BIT(h->hadc_master->State,
              HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY |
              HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP);
    SET_BIT(h->hadc_master->State, HAL_ADC_STATE_READY);

    CLEAR_BIT(h->hadc_slave->State,
              HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY |
              HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP);
    SET_BIT(h->hadc_slave->State, HAL_ADC_STATE_READY);

    PDADC_ResetStreamState(h);

    if (notify_error && (error_cb != NULL)) {
        error_cb(user_ctx);
    }
}
