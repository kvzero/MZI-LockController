/* vim: set ai et ts=4 sw=4: */
#include "stm32g4xx_hal.h"
#include "st7735.h"
#include "string.h"
#include <math.h>
#include <stdlib.h>

#define DELAY 0x80

// based on Adafruit ST7735 library for Arduino
static const uint8_t
  init_cmds1[] = {            // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      ST7735_ROTATION,        //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
  init_cmds2[] = {            // Init for 7735R, part 2 (1.44" display)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127
#endif // ST7735_IS_128X128

#ifdef ST7735_IS_160X80
  init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
    3,                        //  3 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x4F,             //     XEND = 79
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F ,            //     XEND = 159
    ST7735_INVON, 0 },        //  3: Invert colors
#endif

  init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Gamma Adjustments (pos. polarity), 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Gamma Adjustments (neg. polarity), 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay

static void ST7735_Select(ST7735_Handle_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

void ST7735_Unselect(ST7735_Handle_t *dev) {
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static void ST7735_Reset(ST7735_Handle_t *dev) {
    HAL_GPIO_WritePin(dev->res_port, dev->res_pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(dev->res_port, dev->res_pin, GPIO_PIN_SET);
}

static void ST7735_WriteCommand(ST7735_Handle_t *dev, uint8_t cmd) {
    HAL_GPIO_WritePin(dev->dc_port, dev->dc_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(dev->spi, &cmd, sizeof(cmd), HAL_MAX_DELAY);
}

static void ST7735_WriteData(ST7735_Handle_t *dev, uint8_t* buff, size_t buff_size) {
    HAL_GPIO_WritePin(dev->dc_port, dev->dc_pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(dev->spi, buff, buff_size, HAL_MAX_DELAY);
}

static void ST7735_ExecuteCommandList(ST7735_Handle_t *dev, const uint8_t *addr) {
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--) {
        uint8_t cmd = *addr++;
        ST7735_WriteCommand(dev, cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & DELAY;
        numArgs &= ~DELAY;
        if(numArgs) {
            ST7735_WriteData(dev, (uint8_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms) {
            ms = *addr++;
            if(ms == 255) ms = 500;
            HAL_Delay(ms);
        }
    }
}

static void ST7735_SetAddressWindow(ST7735_Handle_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
    // Column address set (CASET)
    int16_t xs = x0 + ST7735_XSTART;
    int16_t xe = x1 + ST7735_XSTART;
    ST7735_WriteCommand(dev, ST7735_CASET);
    uint8_t data_x[] = { xs >> 8, xs & 0xFF, xe >> 8, xe & 0xFF };
    ST7735_WriteData(dev, data_x, sizeof(data_x));

    // Row address set (RASET)
    int16_t ys = y0 + ST7735_YSTART;
    int16_t ye = y1 + ST7735_YSTART;
    ST7735_WriteCommand(dev, ST7735_RASET);
    uint8_t data_y[] = { ys >> 8, ys & 0xFF, ye >> 8, ye & 0xFF };
    ST7735_WriteData(dev, data_y, sizeof(data_y));

    // Write to RAM
    ST7735_WriteCommand(dev, ST7735_RAMWR);
}

void ST7735_Init(ST7735_Handle_t *dev) {
    ST7735_Select(dev);
    ST7735_Reset(dev);
    ST7735_ExecuteCommandList(dev, init_cmds1);
    ST7735_ExecuteCommandList(dev, init_cmds2);
    ST7735_ExecuteCommandList(dev, init_cmds3);
    ST7735_Unselect(dev);
    HAL_TIM_PWM_Start(dev->blk_timer, dev->blk_channel);
    ST7735_SetBrightness(dev, 100);
}

void ST7735_SetBrightness(ST7735_Handle_t *dev, uint8_t brightness)
{
    if (brightness > 100) brightness = 100;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(dev->blk_timer);
    __HAL_TIM_SET_COMPARE(dev->blk_timer, dev->blk_channel, arr * brightness / 100);
}

void ST7735_InvertColors(ST7735_Handle_t *dev, bool invert) {
    ST7735_Select(dev);
    ST7735_WriteCommand(dev, invert ? ST7735_INVON : ST7735_INVOFF);
    ST7735_Unselect(dev);
}

void ST7735_SetGamma(ST7735_Handle_t *dev, GammaDef gamma)
{
	ST7735_Select(dev);
	ST7735_WriteCommand(dev, ST7735_GAMSET);
	ST7735_WriteData(dev, (uint8_t *) &gamma, sizeof(gamma));
	ST7735_Unselect(dev);
}

void ST7735_DrawPixel(ST7735_Handle_t *dev, int16_t x, int16_t y, uint16_t color) {
    if(x < 0 || x >= ST7735_WIDTH || y < 0 || y >= ST7735_HEIGHT) return;

    ST7735_Select(dev);

    ST7735_SetAddressWindow(dev, x, y, x, y);
    uint8_t data[] = { color >> 8, color & 0xFF };
    ST7735_WriteData(dev, data, sizeof(data));

    ST7735_Unselect(dev);
}

void ST7735_FillRectangle(ST7735_Handle_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    // Clipping logic for negative coordinates
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }

    // Clipping logic for exceeding screen boundaries
    if (x + w > ST7735_WIDTH)  w = ST7735_WIDTH - x;
    if (y + h > ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    // Final safety check: if rectangle is completely off-screen
    if (w <= 0 || h <= 0 || x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;

    ST7735_Select(dev);
    ST7735_SetAddressWindow(dev, x, y, x+w-1, y+h-1);

    #define ST7735_FILL_CHUNK_SIZE 256
    uint16_t  chunk[ST7735_FILL_CHUNK_SIZE];
    uint32_t pixels_total = w * h;
    uint16_t pixels_chunk = (pixels_total > ST7735_FILL_CHUNK_SIZE) ? ST7735_FILL_CHUNK_SIZE : pixels_total;
    
    uint16_t color_be = __REV16(color);

    // prepare the chunk buffer
    for(uint16_t i = 0; i < pixels_chunk; i++) {
        chunk[i] = color_be;
    }

    // write to GRAM iteratively
    while(pixels_total > 0) {
        uint16_t send_pixels = (pixels_total > ST7735_FILL_CHUNK_SIZE) ? ST7735_FILL_CHUNK_SIZE : pixels_total;
        ST7735_WriteData(dev, (uint8_t*)chunk, send_pixels * 2);
        pixels_total -= send_pixels;
    }

    ST7735_Unselect(dev);
}

void ST7735_DrawRectangle(ST7735_Handle_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t thick, uint16_t color) {
    // Safety check for thickness
    if (thick == 0) return;
    if (thick > w / 2 || thick > h / 2) {
        ST7735_FillRectangle(dev, x, y, w, h, color);
        return;
    }

    // Draw 4 sides using the optimized FillRectangle
    ST7735_FillRectangle(dev, x, y, w, thick, color);
    ST7735_FillRectangle(dev, x, y + h - thick, w, thick, color);
    ST7735_FillRectangle(dev, x, y + thick, thick, h - 2 * thick, color);
    ST7735_FillRectangle(dev, x + w - thick, y + thick, thick, h - 2 * thick, color);
}

void ST7735_DrawLine(ST7735_Handle_t *dev, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t thick, uint16_t color) {
    // Vertical line optimization with endpoint symmetry
    if (x0 == x1) {
        int16_t y_min = (y0 < y1 ? y0 : y1) - (thick / 2);
        int16_t h_total = (y0 > y1 ? y0 - y1 : y1 - y0) + thick;
        ST7735_FillRectangle(dev, x0 - (thick / 2), y_min, thick, h_total, color);
        return;
    }

    //  Horizontal line optimization with endpoint symmetry
    if (y0 == y1) {
        int16_t x_min = (x0 < x1 ? x0 : x1) - (thick / 2);
        int16_t w_total = (x0 > x1 ? x0 - x1 : x1 - x0) + thick;
        ST7735_FillRectangle(dev, x_min, y0 - (thick / 2), w_total, thick, color);
        return;
    }

    // Bresenham's algorithm for diagonal lines (Square brush approximation)
    int16_t dx = abs(x1 - x0);
    int16_t dy = -abs(y1 - y0);
    int16_t sx = (x0 < x1) ? 1 : -1;
    int16_t sy = (y0 < y1) ? 1 : -1;
    int16_t err = dx + dy, e2;

    while (1) {
        if (thick <= 1) {
            ST7735_DrawPixel(dev, x0, y0, color);
        } else {
            // Note: Diagonal gaps may appear at 45 degrees due to corner-only contact
            ST7735_FillRectangle(dev, x0 - (thick / 2), y0 - (thick / 2), thick, thick, color);
        }

        if (x0 == x1 && y0 == y1) break;

        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}


void ST7735_FillCircle(ST7735_Handle_t *dev, int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t x = 0;
    int16_t y = r;
    int16_t d = 3 - 2 * r;

    while (x <= y) {
        // Draw horizontal scan lines between symmetric points
        ST7735_FillRectangle(dev, x0 - x, y0 + y, 2 * x + 1, 1, color);
        ST7735_FillRectangle(dev, x0 - x, y0 - y, 2 * x + 1, 1, color);
        ST7735_FillRectangle(dev, x0 - y, y0 + x, 2 * y + 1, 1, color);
        ST7735_FillRectangle(dev, x0 - y, y0 - x, 2 * y + 1, 1, color);

        if (d < 0) {
            d = d + 4 * x + 6;
        } else {
            d = d + 4 * (x - y) + 10;
            y--;
        }
        x++;
    }
}

void ST7735_DrawCircle(ST7735_Handle_t *dev, int16_t x0, int16_t y0, uint16_t r, uint16_t thick, uint16_t color) {
    if (thick == 0) return;
    if (thick >= r) {
        ST7735_FillCircle(dev, x0, y0, r, color);
        return;
    }

    int16_t r_in = r - thick;
    uint32_t r_sq = (uint32_t)r * r;
    uint32_t rin_sq = (uint32_t)r_in * r_in;

    for (int16_t y = 0; y <= r; y++) {
        uint32_t y_sq = (uint32_t)y * y;
        int16_t x_out = (int16_t)sqrtf((float)(r_sq - y_sq) + 0.5f);

        if (y > r_in) {
            // Solid caps at the top and bottom of the thick ring
            ST7735_FillRectangle(dev, x0 - x_out, y0 - y, 2 * x_out + 1, 1, color);
            if (y != 0) {
                ST7735_FillRectangle(dev, x0 - x_out, y0 + y, 2 * x_out + 1, 1, color);
            }
        } else {
            // Hollow ring segments
            int16_t x_in = (int16_t)sqrtf((float)(rin_sq - y_sq) + 0.5f);
            int16_t w = x_out - x_in;

            // Compensate for float-to-int truncation to prevent visual gaps
            if (w <= 0) w = 1;

            // Upper half (Right segment start pos fixed: x0 + x_out - w + 1)
            ST7735_FillRectangle(dev, x0 - x_out,         y0 - y, w, 1, color);
            ST7735_FillRectangle(dev, x0 + x_out - w + 1, y0 - y, w, 1, color);

            // Lower half
            if (y != 0) {
                ST7735_FillRectangle(dev, x0 - x_out, y0 + y, w, 1, color);
                ST7735_FillRectangle(dev, x0 + x_out - w + 1, y0 + y, w, 1, color);
            }
        }
    }
}

void ST7735_FillScreen(ST7735_Handle_t *dev, uint16_t color) {
    ST7735_FillRectangle(dev, 0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

static void ST7735_WriteChar(ST7735_Handle_t *dev, uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;
    uint16_t line_buf[font.width];

    // Pre-swap color endianness to Big-Endian for SPI (Cortex-M intrinsic)
    uint16_t c_be = __REV16(color);
    uint16_t bg_be = __REV16(bgcolor);

    ST7735_SetAddressWindow(dev, x, y, x + font.width - 1, y + font.height - 1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            line_buf[j] = ((b << j) & 0x8000) ? c_be : bg_be;
        }
        ST7735_WriteData(dev, (uint8_t*)line_buf, font.width * 2);
    }
}

void ST7735_WriteString(ST7735_Handle_t *dev, int16_t x, int16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
  if(x < 0 || y < 0 || x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;  
  
  ST7735_Select(dev);

    while(*str) {
        if(x + font.width >= ST7735_WIDTH) {
            x = 0;
            y += font.height;
            if(y + font.height >= ST7735_HEIGHT) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        ST7735_WriteChar(dev, x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    ST7735_Unselect(dev);
}

void ST7735_DrawImage(ST7735_Handle_t *dev, int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t* data) {
    if(x < 0 || y < 0 || (x + w) > ST7735_WIDTH || (y + h) > ST7735_HEIGHT) return;

    ST7735_Select(dev);
    ST7735_SetAddressWindow(dev, x, y, x+w-1, y+h-1);
    ST7735_WriteData(dev, (uint8_t*)data, sizeof(uint16_t)*w*h);
    ST7735_Unselect(dev);
}
