#ifndef APP_WIDGETS_H
#define APP_WIDGETS_H

#include <stdint.h>

/**
 * @brief Draw the common page header and clear the screen background.
 */
void APPW_DrawFrame(const char *title);

/**
 * @brief Draw one body text line using the shared page text style.
 */
void APPW_WriteBodyLine(int16_t y, const char *text);

#endif /* APP_WIDGETS_H */
