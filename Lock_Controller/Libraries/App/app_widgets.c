#include "app_widgets.h"
#include <string.h>
#include "app_hardware.h"

void APPW_DrawFrame(const char *title, uint16_t title_fg, uint16_t title_bg)
{
    ST7735_Handle_t *lcd = g_hw->hlcd;
    int16_t title_x;
    uint16_t title_px;

    if (lcd == NULL) {
        return;
    }

    if (title == NULL) {
        title = "";
    }

    title_px = (uint16_t)(strlen(title) * (size_t)Font_11x18.width);
    title_x = (title_px < ST7735_WIDTH) ? (int16_t)((ST7735_WIDTH - title_px) / 2U) : 0;

    ST7735_FillRectangle(lcd, 0, 0, ST7735_WIDTH, 25, title_bg);
    ST7735_WriteString(lcd, title_x, 4, title, Font_11x18, title_fg, title_bg);
}

void APPW_WriteBodyLine(int16_t y, const char *text, uint16_t text_fg)
{
    ST7735_Handle_t *lcd = g_hw->hlcd;

    if (lcd == NULL) {
        return;
    }

    ST7735_WriteString(lcd, 8, y, text, Font_7x10, text_fg, ST7735_BLACK);
}
