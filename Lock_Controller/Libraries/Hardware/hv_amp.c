#include "hv_amp.h"

void HVAMP_Init(HVAMP_Handle_t *h)
{
    /* DAC_Init must be called first to ensure 0V output before enabling */
    HVAMP_Enable(h);
}

void HVAMP_Enable(HVAMP_Handle_t *h)
{
    HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_SET);
    h->enabled = true;

    /* Waiting for OPA462 internal logic, Fault recovers to a typical value of 11us when Disable ->Enable */
    HAL_Delay(1); 
}

void HVAMP_Disable(HVAMP_Handle_t *h)
{
    HAL_GPIO_WritePin(h->en_port, h->en_pin, GPIO_PIN_RESET);
    h->enabled = false;
}

bool HVAMP_IsEnabled(const HVAMP_Handle_t *h)
{
    return h->enabled;
}

bool HVAMP_ReadFault(const HVAMP_Handle_t *h)
{
    /* Status Flag is active-low: GPIO_PIN_RESET means fault */
    return (HAL_GPIO_ReadPin(h->fault_port, h->fault_pin) == GPIO_PIN_RESET);
}
