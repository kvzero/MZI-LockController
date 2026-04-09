#ifndef APP_CORE_H
#define APP_CORE_H

#include "button.h"
#include "app_context.h"

/**
 * @brief Private page contract used by the top-level app dispatcher.
 */
typedef struct {
    void (*enter)(void);
    void (*process)(void);
    void (*render)(void);
    void (*on_button)(Button_Event_t evt);
    void (*exit)(void);
} AppPageOps_t;

extern const AppPageOps_t APP_PAGE_WAIT_OPS;
extern const AppPageOps_t APP_PAGE_ACQUIRE_OPS;
extern const AppPageOps_t APP_PAGE_LOCK_OPS;
extern const AppPageOps_t APP_PAGE_FAULT_OPS;

void APP_GotoPage(AppPage_t page);
void APP_RequestRender(void);

#endif /* APP_CORE_H */
