#ifndef APP_H
#define APP_H

#include "button.h"
#include "app_context.h"

/**
 * @brief Initialise the app runtime and enter the default page.
 */
void APP_Init(void);

/**
 * @brief Run the current page front-end process.
 */
void APP_Process(void);

/**
 * @brief Forward one button event to the active page.
 */
void APP_OnButton(Button_Event_t evt);

/**
 * @brief Render the active page when the UI is marked dirty.
 */
void APP_RenderIfNeeded(void);

/**
 * @brief Enter the fault page with the supplied fault code.
 */
void APP_SetFault(FaultCode_t fault);

#endif /* APP_H */
