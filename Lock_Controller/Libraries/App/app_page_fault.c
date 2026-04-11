#include "app_page_fault.h"
#include "app_core.h"
#include "app_hardware.h"
#include "app_widgets.h"

static void APPFAULT_Render(void);
static const char *APPFAULT_GetFaultText(FaultCode_t fault);
static const char *APPFAULT_GetFaultHint(FaultCode_t fault);

const AppPageOps_t APP_PAGE_FAULT_OPS = {
    .render = APPFAULT_Render,
};

static void APPFAULT_Render(void)
{
    if (g_hw->hlcd != NULL) {
        ST7735_FillScreen(g_hw->hlcd, ST7735_BLACK);
    }

    APPW_DrawFrame("Fault", ST7735_YELLOW, ST7735_RED);
    APPW_WriteBodyLine(38, APPFAULT_GetFaultText(g_rt.fault), ST7735_WHITE);
    APPW_WriteBodyLine(54, APPFAULT_GetFaultHint(g_rt.fault), ST7735_WHITE);
}

static const char *APPFAULT_GetFaultText(FaultCode_t fault)
{
    switch (fault) {
        case APP_FAULT_HVAMP:
            return "HV amplifier fault";

        case APP_FAULT_ADC:
            return "ADC sample failed";

        case APP_FAULT_LOCK:
            return "Lock capture failed";

        case APP_FAULT_CONTRAST:
            return "Contrast too low";

        case APP_FAULT_LIGHT_HIGH:
            return "Light too high";

        case APP_FAULT_REF_LOW:
            return "Ref-light too low";

        case APP_FAULT_NONE:
        default:
            return "Unknown fault";
    }
}

static const char *APPFAULT_GetFaultHint(FaultCode_t fault)
{
    switch (fault) {
        case APP_FAULT_HVAMP:
            return "Check overcurrent or cooling";

        case APP_FAULT_ADC:
            return "Check ADC/DMA path";

        case APP_FAULT_LOCK:
            return "Re-run acquire";

        case APP_FAULT_CONTRAST:
            return "Align optics";

        case APP_FAULT_LIGHT_HIGH:
            return "Turn laser off";

        case APP_FAULT_REF_LOW:
            return "Turn laser on";

        case APP_FAULT_NONE:
        default:
            return "Check setup";
    }
}
