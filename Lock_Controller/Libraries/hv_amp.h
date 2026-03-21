#ifndef HV_AMP_H
#define HV_AMP_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>

/**
 * @brief  Handle for the OPA462 high-voltage amplifier.
 *
 * Populate this struct and pass it to every HVAMP_* function.
 * The Status Flag pin is active-low and must be configured with an internal pull-up.
 */
typedef struct {
    GPIO_TypeDef *en_port;      /* E/D pin port  */
    uint16_t      en_pin;       /* E/D pin mask  */
    GPIO_TypeDef *fault_port;   /* Status Flag port */
    uint16_t      fault_pin;    /* Status Flag pin mask (active-low) */
    bool          enabled;      /* Shadow of the current E/D state */
} HVAMP_Handle_t;

/**
 * @brief  Initialise the amplifier handle.
 * @note   Call after MX_GPIO_Init().
 */
void HVAMP_Init(HVAMP_Handle_t *h);

/**
 * @brief  Enable the OPA462 output stage (drive E/D high).
 */
void HVAMP_Enable(HVAMP_Handle_t *h);

/**
 * @brief  Disable the OPA462 output stage (drive E/D low).
 *         Output impedance rises to ~160kΩ, quiescent current drops ~50%
 */
void HVAMP_Disable(HVAMP_Handle_t *h);

/**
 * @brief  Return the cached enable state (does not read hardware).
 */
bool HVAMP_IsEnabled(const HVAMP_Handle_t *h);

/**
 * @brief  Read the Status Flag pin.
 * @retval true  = fault present (over-current or over-temp)
 * @retval false = normal operation
 */
bool HVAMP_ReadFault(const HVAMP_Handle_t *h);

#endif /* HV_AMP_H */
