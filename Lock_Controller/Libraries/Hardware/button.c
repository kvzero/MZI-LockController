#include "button.h"

/* Private variables ---------------------------------------------------------*/
static Button_Event_t button_fifo[BTN_FIFO_SIZE];
static volatile uint8_t fifo_in = 0;
static volatile uint8_t fifo_out = 0;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Push event to internal FIFO (Thread-safe for single-producer/consumer).
 */
static void fifo_push(ButtonID_t id, ButtonAction_t act)
{
    uint8_t next = (fifo_in + 1) % BTN_FIFO_SIZE;
    if (next != fifo_out) {
        button_fifo[fifo_in].id = id;
        button_fifo[fifo_in].action = act;
        fifo_in = next;
    }
}

/**
 * @brief  Physical read of the currently locked button.
 */
static bool is_locked_pressed(Button_Handle_t *h)
{
    for (uint8_t i = 0; i < h->count; i++) {
        if (h->configs[i].id == h->lock_id) {
            return (HAL_GPIO_ReadPin(h->configs[i].port, h->configs[i].pin) == GPIO_PIN_RESET);
        }
    }
    return false;
}

/**
 * @brief  Check if any configured button is physically pressed.
 */
static bool is_any_pressed(Button_Handle_t *h)
{
    for (uint8_t i = 0; i < h->count; i++) {
        if (HAL_GPIO_ReadPin(h->configs[i].port, h->configs[i].pin) == GPIO_PIN_RESET) {
            return true;
        }
    }
    return false;
}

/* Public functions ----------------------------------------------------------*/

void BTN_Init(Button_Handle_t *h)
{
    h->lock_id  = BTN_ID_NONE;
    h->tick_cnt = 0;
    h->state    = BTN_STATE_IDLE;

    if (h->htim != NULL) {
        /* Clear possible pending interrupts */
        __HAL_TIM_CLEAR_IT(h->htim, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(h->htim);
    }
}

void BTN_Process(Button_Handle_t *h)
{
    /* Physical read depends on whether a button is currently locked */
    bool locked_low = (h->lock_id != BTN_ID_NONE) ? is_locked_pressed(h) : false;

    switch (h->state) {
        case BTN_STATE_IDLE:
            for (uint8_t i = 0; i < h->count; i++) {
                if (HAL_GPIO_ReadPin(h->configs[i].port, h->configs[i].pin) == GPIO_PIN_RESET) {
                    h->lock_id  = h->configs[i].id;
                    h->tick_cnt = 0;
                    h->state    = BTN_STATE_DEBOUNCE;
                    break; /* Exclusive Lock: Stop scanning others */
                }
            }
            break;

        case BTN_STATE_DEBOUNCE:
            if (locked_low) {
                if (++h->tick_cnt >= 2) { /* 20ms confirmed */
                    h->tick_cnt = 0;
                    h->state    = BTN_STATE_ACTIVE;
                }
            } else {
                h->state = BTN_STATE_IDLE; /* False trigger or jitter */
            }
            break;

        case BTN_STATE_ACTIVE:
            if (locked_low) {
                if (h->tick_cnt < 0xFFFFFFFF) h->tick_cnt++;
            } else {
                /* Release detected: Finalise based on tick_cnt and push to FIFO */
                ButtonAction_t act = (h->tick_cnt >= 100) ? BTN_ACT_LONG : BTN_ACT_SHORT;
                fifo_push(h->lock_id, act);
                h->state = BTN_STATE_WAIT_RELEASE;
            }
            break;

        case BTN_STATE_WAIT_RELEASE:
            /* Ensure all physical buttons are released before returning to IDLE */
            if (!is_any_pressed(h)) {
                h->lock_id = BTN_ID_NONE;
                h->state   = BTN_STATE_IDLE;
            }
            break;
            
        default:
            h->state = BTN_STATE_IDLE;
            break;
    }
}

bool BTN_GetEvent(Button_Event_t *out)
{
    if (fifo_in == fifo_out) return false;
    
    *out = button_fifo[fifo_out];
    fifo_out = (fifo_out + 1) % BTN_FIFO_SIZE;
    return true;
}
