/*
 * button.h
 *
 * Handle-based button driver with exclusive-lock state machine.
 * Designed for high-precision systems (e.g., MZ Interferometer).
 */

#ifndef BUTTON_H
#define BUTTON_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Settings */
#define BTN_DEBOUNCE_MS     (2 * 10)  /* 2 samples at 10ms */
#define BTN_LONG_PRESS_MS   1000      /* 100 samples at 10ms */
#define BTN_FIFO_SIZE       4

typedef enum {
    BTN_ID_NONE = 0,
    BTN_ID_UP,
    BTN_ID_DOWN,
    BTN_ID_ENTER
} ButtonID_t;

typedef enum {
    BTN_ACT_SHORT = 0,
    BTN_ACT_LONG
} ButtonAction_t;

/**
 * @brief  Internal State Machine States.
 */
typedef enum {
    BTN_STATE_IDLE = 0,
    BTN_STATE_DEBOUNCE,
    BTN_STATE_ACTIVE,
    BTN_STATE_WAIT_RELEASE
} ButtonState_t;

/**
 * @brief  Button event packet sent to UI layer.
 */
typedef struct {
    ButtonID_t     id;
    ButtonAction_t action;
} Button_Event_t;

/**
 * @brief  Hardware configuration for a single button.
 */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    ButtonID_t    id;
} Button_Config_t;

/**
 * @brief  Button Manager handle.
 */
typedef struct {
    const Button_Config_t* configs;
    uint8_t                count;
    TIM_HandleTypeDef*     htim; 
    
    /* Internal State Machine variables */
    ButtonID_t             lock_id;
    uint32_t               tick_cnt;
    ButtonState_t          state;   /* Current FSM state */
} Button_Handle_t;

/**
 * @brief  Initialise the button manager handle.
 */
void BTN_Init(Button_Handle_t *h);

/**
 * @brief  Core state machine. Must be called every 10ms (e.g., in TIMx ISR).
 * @note   Implements exclusive lock: only the first button pressed is processed.
 */
void BTN_Process(Button_Handle_t *h);

/**
 * @brief  Pop the next button event from the internal FIFO.
 * @retval true  = Event retrieved
 * @retval false = FIFO empty
 */
bool BTN_GetEvent(Button_Event_t *out);

#endif /* BUTTON_H */
