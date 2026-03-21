#ifndef PZT_DAC_H
#define PZT_DAC_H

#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_gpio.h"
#include <stdint.h>

/* DAC8830(3.0V) + OPA462(x50.6) full-scale output voltage */
#define DAC_FULL_SCALE_V    151.8f
#define DAC_RAW_MAX         0xFFFFU

typedef struct {
    SPI_TypeDef  *spi;
    GPIO_TypeDef *cs_port;
    uint32_t      cs_pin;
    uint16_t      max_raw;      /* output clamp, raw counts */
    float         max_voltage;
} HV_DAC_Handle_t;

/**
 * @brief  Initialise the DAC, outputs 0V on completion.
 */
void HVDAC_Init(HV_DAC_Handle_t *h);

/**
 * @brief  Set the output voltage ceiling.
 * @note   Values are clamped to [0.0f, DAC_FULL_SCALE_V].
 */
void HVDAC_SetLimitVoltage(HV_DAC_Handle_t *h, float limit_v);

/**
 * @brief  Set the output voltage.
 * @note   Values are clamped to [0.0f, max_voltage].
 */
void HVDAC_SetVoltage(HV_DAC_Handle_t *h, float target_v);

/**
 * @brief  Write a raw 16-bit value directly to the DAC, max_raw clamp included.
 */
static inline void HVDAC_WriteRaw(HV_DAC_Handle_t *h, uint16_t raw)
{
    if (raw > h->max_raw)
        raw = h->max_raw;

    LL_GPIO_ResetOutputPin(h->cs_port, h->cs_pin);

    while (!LL_SPI_IsActiveFlag_TXE(h->spi)) {}
    LL_SPI_TransmitData16(h->spi, raw);

    /* Wait for RXNE instead of BSY - avoids false-clear between transfers */
    while (!LL_SPI_IsActiveFlag_RXNE(h->spi)) {}
    LL_SPI_ReceiveData16(h->spi);

    LL_GPIO_SetOutputPin(h->cs_port, h->cs_pin);
}

#endif /* PZT_DAC_H */
