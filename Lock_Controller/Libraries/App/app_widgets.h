#ifndef APP_WIDGETS_H
#define APP_WIDGETS_H

#include <stdint.h>
#include "st7735.h"

/**
 * @brief Draw the common page header and clear the screen background.
 */
void APPW_DrawFrame(const char *title, uint16_t title_fg, uint16_t title_bg);

/**
 * @brief Draw one body text line using the shared page text style.
 */
void APPW_WriteBodyLine(int16_t y, const char *text, uint16_t text_fg);

#endif /* APP_WIDGETS_H */
