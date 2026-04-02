/**
  ******************************************************************************
  * @file    dbg_dac.h
  * @brief   Inline driver for the on-chip DAC used as a real-time debug probe.
  ******************************************************************************
  */

#ifndef DBG_DAC_H
#define DBG_DAC_H

#include "stm32g4xx_ll_dac.h"
#include <stdint.h>

#ifndef DBG_DAC_VREF_V
#define DBG_DAC_VREF_V      3.0f
#endif

#define DBG_DAC_INSTANCE    DAC2
#define DBG_DAC_CHANNEL     LL_DAC_CHANNEL_1
#define DBG_DAC_RAW_MAX     0xFFFU      /* 12-bit */

/**
  * @brief  Enable DAC output and drive to 0 V.
  * @note   MX_DACx_Init() must be called before DBG_DAC_Init().
  */
static inline void DBG_DAC_Init(void)
{
    LL_DAC_Enable(DBG_DAC_INSTANCE, DBG_DAC_CHANNEL);
    LL_DAC_ConvertData12RightAligned(DBG_DAC_INSTANCE, DBG_DAC_CHANNEL, 0U);
}

/**
  * @brief  Write a 12-bit raw value to the DAC output.
  * @param  raw   Counts in [0, 4095]; silently clamped.
  */
static inline void DBG_DAC_WriteRaw(uint16_t raw)
{
    if (raw > DBG_DAC_RAW_MAX) raw = DBG_DAC_RAW_MAX;
    LL_DAC_ConvertData12RightAligned(DBG_DAC_INSTANCE, DBG_DAC_CHANNEL, raw);
}

/**
  * @brief  Drive PA6 to the requested voltage.
  * @param  v     Target in [0.0, DBG_DAC_VREF_V]; silently clamped.
  */
static inline void DBG_DAC_SetVoltage(float v)
{
    if (v <= 0.0f) { DBG_DAC_WriteRaw(0U); return; }
    if (v >= DBG_DAC_VREF_V) { DBG_DAC_WriteRaw(DBG_DAC_RAW_MAX); return; }
    DBG_DAC_WriteRaw((uint16_t)(v / DBG_DAC_VREF_V * (float)DBG_DAC_RAW_MAX + 0.5f));
}

#endif /* DBG_DAC_H */
