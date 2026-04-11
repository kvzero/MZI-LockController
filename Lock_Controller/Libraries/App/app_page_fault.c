#include "app_page_fault.h"
#include <stdio.h>
#include "app_core.h"
#include "app_hardware.h"
#include "app_widgets.h"

static void APPFAULT_Render(void);
static const char *APPFAULT_GetFaultText(FaultCode_t fault);

const AppPageOps_t APP_PAGE_FAULT_OPS = {
    .render = APPFAULT_Render,
};

static void APPFAULT_Render(void)
{
    char line[32];

    if (g_hw->hlcd != NULL) {
        ST7735_FillScreen(g_hw->hlcd, ST7735_BLACK);
    }

    APPW_DrawFrame("Fault", ST7735_YELLOW, ST7735_RED);
    APPW_WriteBodyLine(28, APPFAULT_GetFaultText(g_rt.fault), ST7735_YELLOW);

    (void)snprintf(line, sizeof(line), "Code: %u", (unsigned)g_rt.fault);
    APPW_WriteBodyLine(46, line, ST7735_YELLOW);
}

static const char *APPFAULT_GetFaultText(FaultCode_t fault)
{
    switch (fault) {
        case APP_FAULT_HVAMP:
            return "HV amp fault";

        case APP_FAULT_ADC:
            return "ADC sample failed";

        case APP_FAULT_LOCK:
            return "Lock capture failed";

        case APP_FAULT_CONTRAST:
            return "Contrast too low";

        case APP_FAULT_NONE:
        default:
            return "Unknown fault";
    }
}
