#include "app.h"
#include "app_core.h"
#include "app_widgets.h"

static void APPWAIT_Render(void);
static void APPWAIT_OnButton(Button_Event_t evt);

const AppPageOps_t APP_PAGE_WAIT_OPS = {
    .render = APPWAIT_Render,
    .on_button = APPWAIT_OnButton,
};

static void APPWAIT_Render(void)
{
    APPW_DrawFrame("Laser Check");
    APPW_WriteBodyLine(28, "Turn off laser");
    APPW_WriteBodyLine(44, "Short press any key");
}

static void APPWAIT_OnButton(Button_Event_t evt)
{
    if ((evt.id == BTN_ID_NONE) || (evt.action != BTN_ACT_SHORT)) {
        return;
    }

    APP_GotoPage(APP_PAGE_ACQUIRE);
}
