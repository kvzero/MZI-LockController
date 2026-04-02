#include "hv_dac.h"

static inline uint16_t voltage_to_raw(float v)
{
    return (uint16_t)((v / DAC_FULL_SCALE_V) * (float)DAC_RAW_MAX + 0.5f);
}

void HVDAC_Init(HV_DAC_Handle_t *h)
{
    if (h->max_voltage <= 0.0f)
        h->max_voltage = DAC_FULL_SCALE_V;

    h->max_raw = voltage_to_raw(h->max_voltage);

    if (!LL_SPI_IsEnabled(h->spi))
        LL_SPI_Enable(h->spi);

    /* CS high before touching the bus */
    LL_GPIO_SetOutputPin(h->cs_port, h->cs_pin);

    /* Flush any residual RX data */
    while (LL_SPI_IsActiveFlag_RXNE(h->spi))
        LL_SPI_ReceiveData16(h->spi);

    HVDAC_WriteRaw(h, 0U);
}

void HVDAC_SetLimitVoltage(HV_DAC_Handle_t *h, float limit_v)
{
    if (limit_v > DAC_FULL_SCALE_V) limit_v = DAC_FULL_SCALE_V;
    if (limit_v < 0.0f)             limit_v = 0.0f;

    h->max_voltage = limit_v;
    h->max_raw     = voltage_to_raw(limit_v);
}

void HVDAC_SetVoltage(HV_DAC_Handle_t *h, float target_v)
{
    if (target_v > h->max_voltage) target_v = h->max_voltage;
    if (target_v < 0.0f)           target_v = 0.0f;

    HVDAC_WriteRaw(h, voltage_to_raw(target_v));
}
