#include "app_widgets.h"
#include "app_hardware.h"

void APPW_DrawFrame(const char *title)
{
    ST7735_Handle_t *lcd = g_hw->hlcd;

    if (lcd == NULL) {
        return;
    }

    ST7735_FillScreen(lcd, ST7735_BLACK);
    ST7735_FillRectangle(lcd, 0, 0, ST7735_WIDTH, 20, ST7735_WHITE);
    ST7735_WriteString(lcd, 6, 4, title, Font_11x18, ST7735_BLACK, ST7735_WHITE);
}

void APPW_WriteBodyLine(int16_t y, const char *text)
{
    ST7735_Handle_t *lcd = g_hw->hlcd;

    if (lcd == NULL) {
        return;
    }

    ST7735_WriteString(lcd, 8, y, text, Font_7x10, ST7735_WHITE, ST7735_BLACK);
}
