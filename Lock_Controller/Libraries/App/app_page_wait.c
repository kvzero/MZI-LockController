#include "app_page_wait.h"
#include "app.h"
#include "app_core.h"
#include "app_hardware.h"
#include "app_widgets.h"

static void APPWAIT_Render(void);
static void APPWAIT_OnButton(Button_Event_t evt);

const AppPageOps_t APP_PAGE_WAIT_OPS = {
    .render = APPWAIT_Render,
    .on_button = APPWAIT_OnButton,
};

static void APPWAIT_Render(void)
{
    if (g_hw->hlcd != NULL) {
        ST7735_FillScreen(g_hw->hlcd, ST7735_BLACK);
    }

    APPW_DrawFrame("Laser Check", ST7735_BLACK, ST7735_WHITE);
    APPW_WriteBodyLine(38, "Turn off the laser", ST7735_WHITE);
    APPW_WriteBodyLine(54, "Short press any key", ST7735_WHITE);
}

static void APPWAIT_OnButton(Button_Event_t evt)
{
    if ((evt.id == BTN_ID_NONE) || (evt.action != BTN_ACT_SHORT)) {
        return;
    }

    APP_GotoPage(APP_PAGE_ACQUIRE);
}
