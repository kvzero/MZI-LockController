#ifndef APP_PAGE_H
#define APP_PAGE_H

#include "button.h"

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

#endif /* APP_PAGE_H */
