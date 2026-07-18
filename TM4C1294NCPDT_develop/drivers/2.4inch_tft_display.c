//*****************************************************************************
// C file to handle a 2.4inch_TFT_display, 240x320px featuring an 
// ili9341 display driver and xpt2046 touch controller.
//*****************************************************************************
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
#include "drivers/2.4inch_tft_display.h"
// Delay library
#include "delay.h"
//*****************************************************************************
// VARIABLES
//*****************************************************************************
// Must be definen on .c file which call this library
extern uint32_t ui32SysClock;
// For uDMA use 
#pragma DATA_ALIGN(uDMAControlTable, 1024);
uint8_t uDMAControlTable[1024];
static uint16_t colorBufer[ILI9341_WIDTH * ILI9341_LINES_PER_BUFFER];
static uint16_t g_ui16CharBuf[FONT_WIDTH * FONT_HEIGHT];
// Commands and data to init display driver
typedef struct {
    uint8_t  cmd;
    uint8_t  numArgs;
    uint8_t  args[15];
    uint16_t delayMs;
} ILI9341_InitCmd;
static const ILI9341_InitCmd ili9341_init_seq[] = {
    { 0xCB, 5, { 0x39, 0x2C, 0x00, 0x34, 0x02 },          0 }, // Power control A
    { 0xCF, 3, { 0x00, 0xC1, 0x30 },                      0 }, // Power control B
    { 0xE8, 3, { 0x85, 0x00, 0x78 },                      0 }, // Driver timing control A
    { 0xEA, 2, { 0x00, 0x00 },                            0 }, // Driver timing control B
    { 0xED, 4, { 0x64, 0x03, 0x12, 0x81 },                0 }, // Power-on sequence control
    { 0xF7, 1, { 0x20 },                                  0 }, // Pump ratio control
    { 0xC0, 1, { 0x23 },                                  0 }, // Power control 1 (VRH)
    { 0xC1, 1, { 0x10 },                                  0 }, // Power control 2 (BT)
    { 0xC5, 2, { 0x3E, 0x28 },                            0 }, // VCOM control 1
    { 0xC7, 1, { 0x86 },                                  0 }, // VCOM control 2
    { ILI9341_COLMOD, 1, { 0x55 },                        0 }, // 16 bpp, RGB565
    { ILI9341_MADCTL, 1, { 0x20 },                        0 }, // RGB, 180 deg rotation
    { 0xB1, 2, { 0x00, 0x10 },                            0 }, // Frame rate ~119 Hz (max); use 0x18 (~79 Hz) if unstable
    { 0xB6, 3, { 0x08, 0x82, 0x27 },                      0 }, // Display function control (320 lines)
    { 0xF2, 1, { 0x00 },                                  0 }, // 3-gamma disable
    { 0x26, 1, { 0x01 },                                  0 }, // Gamma curve 1
    { 0xE0, 15, { 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
                  0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00 }, 0 }, // Positive gamma
    { 0xE1, 15, { 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
                  0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F }, 0 }, // Negative gamma
    { ILI9341_SLPOUT, 0, { 0 },                         120 }, // Sleep out, then supply/clock settle
    { ILI9341_DISPON, 0, { 0 },                          20 }, // Display on
};
// Display font
static const unsigned char font8x16[][16] = { 
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x20, ' '
        { 0x00, 0x00, 0x18, 0x3C, 0x3C, 0x3C, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,  },       //0x21, '!'
        { 0x00, 0x66, 0x66, 0x66, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x22, '"'
        { 0x00, 0x00, 0x00, 0x6C, 0x6C, 0xFE, 0x6C, 0x6C, 0x6C, 0xFE, 0x6C, 0x6C, 0x00, 0x00, 0x00, 0x00,  },       //0x23, '#'
        { 0x18, 0x18, 0x7C, 0xC6, 0xC2, 0xC0, 0x7C, 0x06, 0x06, 0x86, 0xC6, 0x7C, 0x18, 0x18, 0x00, 0x00,  },       //0x24, '$'
        { 0x00, 0x00, 0x00, 0x00, 0xC2, 0xC6, 0x0C, 0x18, 0x30, 0x60, 0xC6, 0x86, 0x00, 0x00, 0x00, 0x00,  },       //0x25, '%'
        { 0x00, 0x00, 0x38, 0x6C, 0x6C, 0x38, 0x76, 0xDC, 0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00,  },       //0x26, '&'
        { 0x00, 0x30, 0x30, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x27, '''
        { 0x00, 0x00, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x18, 0x0C, 0x00, 0x00, 0x00, 0x00,  },       //0x28, '('
        { 0x00, 0x00, 0x30, 0x18, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00,  },       //0x29, ')'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x3C, 0xFF, 0x3C, 0x66, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x2A, '*'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x7E, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x2B, '+'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00,  },       //0x2C, '
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x2D, '-'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,  },       //0x2E, '.'
        { 0x00, 0x00, 0x00, 0x00, 0x02, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0x80, 0x00, 0x00, 0x00, 0x00,  },       //0x2F, '/'
        { 0x00, 0x00, 0x38, 0x6C, 0xC6, 0xC6, 0xD6, 0xD6, 0xC6, 0xC6, 0x6C, 0x38, 0x00, 0x00, 0x00, 0x00,  },       //0x30, '0'
        { 0x00, 0x00, 0x18, 0x38, 0x78, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x7E, 0x00, 0x00, 0x00, 0x00,  },       //0x31, '1'
        { 0x00, 0x00, 0x7C, 0xC6, 0x06, 0x0C, 0x18, 0x30, 0x60, 0xC0, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00,  },       //0x32, '2'
        { 0x00, 0x00, 0x7C, 0xC6, 0x06, 0x06, 0x3C, 0x06, 0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x33, '3'
        { 0x00, 0x00, 0x0C, 0x1C, 0x3C, 0x6C, 0xCC, 0xFE, 0x0C, 0x0C, 0x0C, 0x1E, 0x00, 0x00, 0x00, 0x00,  },       //0x34, '4'
        { 0x00, 0x00, 0xFE, 0xC0, 0xC0, 0xC0, 0xFC, 0x06, 0x06, 0x06, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x35, '5'
        { 0x00, 0x00, 0x38, 0x60, 0xC0, 0xC0, 0xFC, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x36, '6'
        { 0x00, 0x00, 0xFE, 0xC6, 0x06, 0x06, 0x0C, 0x18, 0x30, 0x30, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00,  },       //0x37, '7'
        { 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x38, '8'
        { 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0x06, 0x06, 0x0C, 0x78, 0x00, 0x00, 0x00, 0x00,  },       //0x39, '9'
        { 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x3A, ':'
        { 0x00, 0x00, 0x00, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x18, 0x18, 0x30, 0x00, 0x00, 0x00, 0x00,  },       //0x3B, ';'
        { 0x00, 0x00, 0x00, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x00, 0x00, 0x00, 0x00,  },       //0x3C, '<'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x3D, '='
        { 0x00, 0x00, 0x00, 0x60, 0x30, 0x18, 0x0C, 0x06, 0x0C, 0x18, 0x30, 0x60, 0x00, 0x00, 0x00, 0x00,  },       //0x3E, '>'
        { 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0x0C, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,  },       //0x3F, '?'
        { 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xDE, 0xDE, 0xDE, 0xDC, 0xC0, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x40, '@'
        { 0x00, 0x00, 0x10, 0x38, 0x6C, 0xC6, 0xC6, 0xFE, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00,  },       //0x41, 'A'
        { 0x00, 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x66, 0x66, 0x66, 0x66, 0xFC, 0x00, 0x00, 0x00, 0x00,  },       //0x42, 'B'
        { 0x00, 0x00, 0x3C, 0x66, 0xC2, 0xC0, 0xC0, 0xC0, 0xC0, 0xC2, 0x66, 0x3C, 0x00, 0x00, 0x00, 0x00,  },       //0x43, 'C'
        { 0x00, 0x00, 0xF8, 0x6C, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x6C, 0xF8, 0x00, 0x00, 0x00, 0x00,  },       //0x44, 'D'
        { 0x00, 0x00, 0xFE, 0x66, 0x62, 0x68, 0x78, 0x68, 0x60, 0x62, 0x66, 0xFE, 0x00, 0x00, 0x00, 0x00,  },       //0x45, 'E'
        { 0x00, 0x00, 0xFE, 0x66, 0x62, 0x68, 0x78, 0x68, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00, 0x00,  },       //0x46, 'F'
        { 0x00, 0x00, 0x3C, 0x66, 0xC2, 0xC0, 0xC0, 0xDE, 0xC6, 0xC6, 0x66, 0x3A, 0x00, 0x00, 0x00, 0x00,  },       //0x47, 'G'
        { 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xFE, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00,  },       //0x48, 'H'
        { 0x00, 0x00, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00,  },       //0x49, 'I'
        { 0x00, 0x00, 0x1E, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0xCC, 0xCC, 0xCC, 0x78, 0x00, 0x00, 0x00, 0x00,  },       //0x4A, 'J'
        { 0x00, 0x00, 0xE6, 0x66, 0x66, 0x6C, 0x78, 0x78, 0x6C, 0x66, 0x66, 0xE6, 0x00, 0x00, 0x00, 0x00,  },       //0x4B, 'K'
        { 0x00, 0x00, 0xF0, 0x60, 0x60, 0x60, 0x60, 0x60, 0x60, 0x62, 0x66, 0xFE, 0x00, 0x00, 0x00, 0x00,  },       //0x4C, 'L'
        { 0x00, 0x00, 0xC6, 0xEE, 0xFE, 0xFE, 0xD6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00,  },       //0x4D, 'M'
        { 0x00, 0x00, 0xC6, 0xE6, 0xF6, 0xFE, 0xDE, 0xCE, 0xC6, 0xC6, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00,  },       //0x4E, 'N'
        { 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x4F, 'O'
        { 0x00, 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x60, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00, 0x00,  },       //0x50, 'P'
        { 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xD6, 0xDE, 0x7C, 0x0C, 0x0E, 0x00, 0x00,  },       //0x51, 'Q'
        { 0x00, 0x00, 0xFC, 0x66, 0x66, 0x66, 0x7C, 0x6C, 0x66, 0x66, 0x66, 0xE6, 0x00, 0x00, 0x00, 0x00,  },       //0x52, 'R'
        { 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0x60, 0x38, 0x0C, 0x06, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x53, 'S'
        { 0x00, 0x00, 0x7E, 0x7E, 0x5A, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00,  },       //0x54, 'T'
        { 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x55, 'U'
        { 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x6C, 0x38, 0x10, 0x00, 0x00, 0x00, 0x00,  },       //0x56, 'V'
        { 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xD6, 0xD6, 0xD6, 0xFE, 0xEE, 0x6C, 0x00, 0x00, 0x00, 0x00,  },       //0x57, 'W'
        { 0x00, 0x00, 0xC6, 0xC6, 0x6C, 0x7C, 0x38, 0x38, 0x7C, 0x6C, 0xC6, 0xC6, 0x00, 0x00, 0x00, 0x00,  },       //0x58, 'X'
        { 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00,  },       //0x59, 'Y'
        { 0x00, 0x00, 0xFE, 0xC6, 0x86, 0x0C, 0x18, 0x30, 0x60, 0xC2, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00,  },       //0x5A, 'Z'
        { 0x00, 0x00, 0x3C, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x3C, 0x00, 0x00, 0x00, 0x00,  },       //0x5B, '['
        { 0x00, 0x00, 0x00, 0x80, 0xC0, 0xE0, 0x70, 0x38, 0x1C, 0x0E, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00,  },       //0x5C, '\'
        { 0x00, 0x00, 0x3C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x3C, 0x00, 0x00, 0x00, 0x00,  },       //0x5D, ']'
        { 0x10, 0x38, 0x6C, 0xC6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x5E, '^'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00,  },       //0x5F, '_'
        { 0x30, 0x30, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x60, '`'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x0C, 0x7C, 0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00,  },       //0x61, 'a'
        { 0x00, 0x00, 0xE0, 0x60, 0x60, 0x78, 0x6C, 0x66, 0x66, 0x66, 0x66, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x62, 'b'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC0, 0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x63, 'c'
        { 0x00, 0x00, 0x1C, 0x0C, 0x0C, 0x3C, 0x6C, 0xCC, 0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00,  },       //0x64, 'd'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xFE, 0xC0, 0xC0, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x65, 'e'
        { 0x00, 0x00, 0x38, 0x6C, 0x64, 0x60, 0xF0, 0x60, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00, 0x00,  },       //0x66, 'f'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x7C, 0x0C, 0xCC, 0x78, 0x00,  },       //0x67, 'g'
        { 0x00, 0x00, 0xE0, 0x60, 0x60, 0x6C, 0x76, 0x66, 0x66, 0x66, 0x66, 0xE6, 0x00, 0x00, 0x00, 0x00,  },       //0x68, 'h'
        { 0x00, 0x00, 0x18, 0x18, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00,  },       //0x69, 'i'
        { 0x00, 0x00, 0x06, 0x06, 0x00, 0x0E, 0x06, 0x06, 0x06, 0x06, 0x06, 0x06, 0x66, 0x66, 0x3C, 0x00,  },       //0x6A, 'j'
        { 0x00, 0x00, 0xE0, 0x60, 0x60, 0x66, 0x6C, 0x78, 0x78, 0x6C, 0x66, 0xE6, 0x00, 0x00, 0x00, 0x00,  },       //0x6B, 'k'
        { 0x00, 0x00, 0x38, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x00, 0x00, 0x00, 0x00,  },       //0x6C, 'l'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0xEC, 0xFE, 0xD6, 0xD6, 0xD6, 0xD6, 0xC6, 0x00, 0x00, 0x00, 0x00,  },       //0x6D, 'm'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x00, 0x00, 0x00, 0x00,  },       //0x6E, 'n'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x6F, 'o'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x66, 0x66, 0x66, 0x66, 0x66, 0x7C, 0x60, 0x60, 0xF0, 0x00,  },       //0x70, 'p'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x76, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x7C, 0x0C, 0x0C, 0x1E, 0x00,  },       //0x71, 'q'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0xDC, 0x76, 0x66, 0x60, 0x60, 0x60, 0xF0, 0x00, 0x00, 0x00, 0x00,  },       //0x72, 'r'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 0xC6, 0x60, 0x38, 0x0C, 0xC6, 0x7C, 0x00, 0x00, 0x00, 0x00,  },       //0x73, 's'
        { 0x00, 0x00, 0x10, 0x30, 0x30, 0xFC, 0x30, 0x30, 0x30, 0x30, 0x36, 0x1C, 0x00, 0x00, 0x00, 0x00,  },       //0x74, 't'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0x76, 0x00, 0x00, 0x00, 0x00,  },       //0x75, 'u'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x66, 0x66, 0x66, 0x66, 0x66, 0x3C, 0x18, 0x00, 0x00, 0x00, 0x00,  },       //0x76, 'v'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xD6, 0xD6, 0xD6, 0xFE, 0x6C, 0x00, 0x00, 0x00, 0x00,  },       //0x77, 'w'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0x6C, 0x38, 0x38, 0x38, 0x6C, 0xC6, 0x00, 0x00, 0x00, 0x00,  },       //0x78, 'x'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0xC6, 0x7E, 0x06, 0x0C, 0xF8, 0x00,  },       //0x79, 'y'
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xCC, 0x18, 0x30, 0x60, 0xC6, 0xFE, 0x00, 0x00, 0x00, 0x00,  },       //0x7A, 'z'
        { 0x00, 0x00, 0x0E, 0x18, 0x18, 0x18, 0x70, 0x18, 0x18, 0x18, 0x18, 0x0E, 0x00, 0x00, 0x00, 0x00,  },       //0x7B, '{'
        { 0x00, 0x00, 0x18, 0x18, 0x18, 0x18, 0x00, 0x18, 0x18, 0x18, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00,  },       //0x7C, '|'
        { 0x00, 0x00, 0x70, 0x18, 0x18, 0x18, 0x0E, 0x18, 0x18, 0x18, 0x18, 0x70, 0x00, 0x00, 0x00, 0x00,  },       //0x7D, '}'
        { 0x00, 0x00, 0x76, 0xDC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  },       //0x7E, '~'
};
// PWM and ADC for display backlight brightness control.
// Latest raw ADC0 SS3 sample (0..4095). Written by the ISR, read by the app.
volatile uint32_t adc0Ssq3Value = 0x0000;
// Set to 1 by the ADC ISR when a new sample is ready; cleared by the consumer.
volatile uint8_t adc_ready = 0x00;
// Backlight duty cycle, 0.0..1.0. Updated from the ADC reading in the ISR.
volatile float fDutyCycle = 0.50f;   // 50%
// PWM period in clock ticks, computed at runtime from the system clock.
static uint32_t pwmLoad = 0x0000;
// PWM pulse width in clock ticks (derived from fDutyCycle).
static uint32_t ui32Width = 0x0000;
// Touch state, private to this driver and exposed through accessor functions.
// Set by the PENIRQ ISR when the panel is pressed; cleared by touch_pressed().
static volatile uint8_t touch_asserted = 0x00;
// Last touch position in screen pixels, updated by touch_request_coords().
static uint16_t touch_x = 0x0000;
static uint16_t touch_y = 0x0000;

//*****************************************************************************
// Public display functions definitions
//*****************************************************************************
void display_init(){
    // Peripherals: SPI link, uDMA pixel pump, and the ADC/PWM backlight control.
    delay_init(ui32SysClock);
    display_spi_config();
    display_dma_config();
    display_adc_config();
    display_pwm_config(); 
    touch_init();

    // Hardware reset (reloads factory defaults; includes the required delays).
    display_rst();

    // Commands and their parameters are single bytes.
    display_spi_data_len(8);

    uint32_t i;
    uint8_t  a;
    for (i = 0; i < sizeof(ili9341_init_seq) / sizeof(ili9341_init_seq[0]); i++){
        const ILI9341_InitCmd *c = &ili9341_init_seq[i];
        display_snd_cmd(c->cmd);
        for (a = 0; a < c->numArgs; a++)
            display_snd_data(c->args[a]);
        if (c->delayMs)
            delay_ms(c->delayMs);
    }

    // Switch to 16-bit frames for fast pixel streaming via uDMA.
    display_spi_data_len(16);
}

void display_rst(void){
    // Assert RESX low (datasheet min 10 us; 10 ms gives generous margin).
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x00);
    delay_ms(10);
    // Release reset; the controller reloads its factory defaults.
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);
    delay_ms(120); // datasheet: wait before the first command
}

void display_enable(void){
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0 , 0x00);
}

void display_disable(void){
    MAP_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_0, GPIO_PIN_0);
}

void display_snd_cmd(uint8_t cmd){
    display_enable();
    // Put the display on command mode
    MAP_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00);
    // Send the command through SSI2
    MAP_SSIDataPut(SSI3_BASE, cmd);
    // Wait for data to be transferred
    while(MAP_SSIBusy(SSI3_BASE));
    display_disable();
}

void display_snd_data(uint8_t data){
    display_enable();
    // Put the display on data mode
    MAP_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1, GPIO_PIN_1);
    // Send the data to SSI2
    MAP_SSIDataPut(SSI3_BASE, data);
    // Wait for data to be transferred
    while(MAP_SSIBusy(SSI3_BASE));
    display_disable();
}

void display_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1){
    // Config spi3 to send command and data configuration
    display_spi_data_len(8);
    // Column Address Set (CASET)
    display_snd_cmd(ILI9341_CASET);
    display_snd_data(x0 >> 8);     // X start high
    display_snd_data(x0 & 0xFF);   // X start low
    display_snd_data(x1 >> 8);     // X end high
    display_snd_data(x1 & 0xFF);   // X end low

    // Page Address Set (PASET)
    display_snd_cmd(ILI9341_PASET);
    display_snd_data(y0 >> 8);     // Y start high
    display_snd_data(y0 & 0xFF);   // Y start low
    display_snd_data(y1 >> 8);     // Y end high
    display_snd_data(y1 & 0xFF);   // Y end low

    // Write to RAM
    display_snd_cmd(ILI9341_RAMWR);
    // Config spi3 to send pixels data
    display_spi_data_len(16);
}
void display_fill_screen(uint16_t color){
    uint16_t  i;
    // Fill a chunk color buffer
    for(i=0;i<ILI9341_WIDTH*ILI9341_LINES_PER_BUFFER;i++)
        colorBufer[i] = color;
    
    display_set_window(0, 0, ILI9341_WIDTH - 1, ILI9341_HEIGHT - 1);
    display_enable();
    // Put display on data mode
    MAP_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1, GPIO_PIN_1);
    for (i = 0; i < ILI9341_HEIGHT; i+=ILI9341_LINES_PER_BUFFER){
        /*
         * Wait only for DMA done — NOT for SSIBusy.
         * The FIFO keeps clocking bits out in the background while we poll.
         * The next DMA burst starts feeding the FIFO as soon as it has room,
         * with zero dead time on the SPI bus.
         */
        while (MAP_uDMAChannelSizeGet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT));

        display_snd_dma_buffer(colorBufer, ILI9341_WIDTH * ILI9341_LINES_PER_BUFFER);
    }

    /* Wait for the shift register to drain only once, at the very end */
    while (SSIBusy(SSI3_BASE));
}
void display_draw_char(uint16_t x, uint16_t y,char ch,uint16_t fgColor, uint16_t bgColor){
    // Usefull variables to fill the buffer
    uint8_t   row, col;
    uint8_t   fontIdx;
    uint8_t   fontByte;
    uint16_t *pPix = g_ui16CharBuf;

    // ── Bounds check ─────────────────────────────────────────────────────────
    // A character starting at x=236 would write pixels 236..243,
    // but the screen only goes to 239 → corrupts ILI9341 address counter.
    if (x + FONT_WIDTH > ILI9341_WIDTH)  return;
    if (y + FONT_HEIGHT > ILI9341_HEIGHT) return;

    // ── ASCII → font table index ─────────────────────────────────────────────
    // Your table starts at 0x20 (space), so:
    //   ' '  → index 0
    //   '!'  → index 1
    //   'A'  → index 33   (0x41 - 0x20)
    if (ch < (char)FONT_FIRST_CHAR || ch > (char)FONT_LAST_CHAR)
        ch = (char)FONT_FALLBACK;
    fontIdx = (uint8_t)((uint8_t)ch - FONT_FIRST_CHAR);

    // ── Expand bitmap → RGB565 pixels ─────────────────────────────────────────
    // Outer loop: rows top → bottom (row 0 = topmost screen row of glyph)
    // Inner loop: columns left → right
    // Memory layout in g_ui16CharBuf:
    //   [0..7]   = row 0  (top row of glyph,   8 pixels)
    //   [8..15]  = row 1
    //   ...
    //   [120..127] = row 15 (bottom row of glyph)
    // This is exactly the order ILI9341 expects after RAMWR
    // because we set a top-to-bottom window.
    for (row = 0; row < FONT_HEIGHT; row++)
    {
        fontByte = font8x16[fontIdx][row];

        for (col = 0; col < FONT_WIDTH; col++)
        {
            // Shift mask from MSB down: col0→bit7, col1→bit6 ... col7→bit0
            *pPix++ = (fontByte & (0x80u >> col)) ? fgColor : bgColor;
        }
    }
    // ── Set 8×16 window, fire DMA ─────────────────────────────────────────────
    // Window = exactly the character bounding box.
    // ILI9341 will auto-advance within this window — no address math needed.
    display_set_window(x,y,x + FONT_WIDTH - 1, y + FONT_HEIGHT - 1);
    display_enable();
    // Put the display on data mode
    MAP_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1, GPIO_PIN_1);
    // Kick off DMA: 128 pixels × 16-bit = 256 bytes
    display_snd_dma_buffer(g_ui16CharBuf, FONT_WIDTH * FONT_HEIGHT);
    // Must wait for BOTH DMA done AND shift register empty before
    // the next ili9341_set_window() call. set_window sends SPI commands
    // and if SSI is still shifting pixels you corrupt the pixel stream.
    while (MAP_uDMAChannelSizeGet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT));
    while (SSIBusy(SSI3_BASE));
    display_disable();
}
void display_print_string(uint16_t x, uint16_t y,const char *str,uint16_t fgColor,uint16_t bgColor){
    uint16_t curX = x;
    uint16_t curY = y;

    while (*str != '\0'){
        char ch = *str++;

        // ── Newline ───────────────────────────────────────────────────────────
        if (ch == '\n'){
            curX  = x;          // Return to left margin (not column 0 — respects x)
            curY += FONT_HEIGHT;     // Drop one text row (16 pixels)
            if (curY + FONT_HEIGHT > ILI9341_HEIGHT) return;   // Off bottom → stop
            continue;
        }

        if (ch == '\r') continue;   // Ignore bare carriage return

        // ── Auto word-wrap ───────────────────────────────────────────────────
        // Don't let a character start where it would run past the right edge
        if (curX + FONT_WIDTH > ILI9341_WIDTH){
            curX  = x;
            curY += FONT_HEIGHT;
            if (curY + FONT_HEIGHT > ILI9341_HEIGHT) return;
        }

        display_draw_char(curX, curY, ch, fgColor, bgColor);
        curX += FONT_WIDTH;    // Advance cursor one glyph width
    }
}
void display_print_int(uint16_t x, uint16_t y,int32_t num,uint16_t color, uint16_t bg){
    char buf[12];   // "-2147483648" + '\0' = 12 bytes
    intToStr(num, buf);
    display_print_string(x,y,buf,color,bg);
}
void display_print_float(uint16_t x, uint16_t y,float num, uint8_t decimals,uint16_t color, uint16_t bg){
    // '-' + up to 10 integer digits + '.' + decimals + '\0'.
    // Clamp decimals so the result always fits the buffer.
    char buf[24];
    if (decimals > 9) decimals = 9;
    floatToStr(num, buf,decimals);
    display_print_string(x,y,buf,color,bg);
}
//*****************************************************************************
// Support functions definitions
//*****************************************************************************
static void intToStr(int32_t value, char *buf){
    char tmp[12];
    int i = 0, j = 0;

    if (value == 0)
    {
        buf[0] = '0';
        buf[1] = '\0';
        return;
    }

    if (value < 0)
    {
        buf[j++] = '-';
        value = -value;
    }

    while (value > 0)
    {
        tmp[i++] = (value % 10) + '0';
        value /= 10;
    }

    while (i > 0)
        buf[j++] = tmp[--i];

    buf[j] = '\0';
}

static void floatToStr(float value, char *buf, uint8_t decimals){
    int32_t intPart;
    float frac;
    int i = 0, j = 0;
    uint8_t d;

    if (value < 0.0f)
    {
        buf[i++] = '-';
        value = -value;
    }

    // Round to the requested number of decimals so the last digit is correct
    // (e.g. 1.999 with 2 decimals -> "2.00" instead of truncating to "1.99").
    // The carry from rounding may also bump the integer part, so do it first.
    float rounding = 0.5f;
    for (d = 0; d < decimals; d++)
        rounding *= 0.1f;
    value += rounding;

    intPart = (int32_t)value;
    frac = value - (float)intPart;

    // Integer part
    char intBuf[12];
    intToStr(intPart, intBuf);

    for (j = 0; intBuf[j]; j++)
        buf[i++] = intBuf[j];

    // Fractional part (only when decimals were requested)
    if (decimals > 0)
    {
        buf[i++] = '.';
        while (decimals--)
        {
            frac *= 10.0f;
            int digit = (int)frac;
            if (digit > 9) digit = 9;       // guard against fp rounding to 10
            buf[i++] = (char)('0' + digit);
            frac -= digit;
        }
    }

    buf[i] = '\0';
}
//*****************************************************************************
// Related peripheral config functions
//*****************************************************************************
// ADC
void display_adc_config(void){
    // Enable the ADC module and related peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    // Wait for the ADC0 module to be ready
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    // Disable digital function on the pin
    MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Config ADC clock
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,12);
    // Use VDDA (3.3 V)
    MAP_ADCReferenceSet(ADC0_BASE, ADC_REF_INT);
    // Configure ADC oversampling
    MAP_ADCHardwareOversampleConfigure(ADC0_BASE, 0);
    // Disable sequencer 3 during config
    MAP_ADCSequenceDisable(ADC0_BASE, 3);
    // Enable the first sample sequencer to capture the value of channel 0 when the processor trigger occurs
    MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PWM_MOD0 | ADC_TRIGGER_PWM1, 0);
    // Configure sequencer steps
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    // Clear ADC0 Sequencer 3 interrupt
    MAP_ADCIntClear(ADC0_BASE, 3);
    // Enable ADC0 Sequencer 3 interrupt
    MAP_ADCIntEnable(ADC0_BASE, 3);
    // Enable interrupt in NVIC
    MAP_IntEnable(INT_ADC0SS3);
    // Enable ADC sequencer
    MAP_ADCSequenceEnable(ADC0_BASE, 3);
}

// ADC0 Sequencer 3 interrupt handler
void display_adc_handler(void){
    uint32_t sample = 0;
    // ALWAYS clear first, before reading
    MAP_ADCIntClear(ADC0_BASE, 3);
    // Read ADC result into a non-volatile local, then publish it (avoids
    // passing a volatile pointer to the DriverLib API).
    MAP_ADCSequenceDataGet(ADC0_BASE, 3, &sample);
    adc0Ssq3Value = sample;
    // Update PWM duty cycle based on ADC reading (0-4095 -> 0-100%)
    fDutyCycle = (float)sample / 4095.0f;
    ui32Width  = (uint32_t)(pwmLoad * fDutyCycle);
    // Clamp: PWM hardware requires 1 <= width <= (pwmLoad - 1)
    // If width == 0 or >= pwmLoad the comparator never fires -> output glitches
    if(ui32Width < 1)           ui32Width = 1;
    if(ui32Width > pwmLoad - 1) ui32Width = pwmLoad - 1;
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Width);
    // Signal main loop that a new ADC value is ready
    adc_ready = 0x01;
}

// PWM
void display_pwm_config(){
    // Enabling and waiting for related peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Configure pin PM3 as PWM output
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    // Config PWM module clock
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_2);
    // PWM period in ticks, derived from the system clock (PWM_SYSCLK_DIV_2 halves it)
    pwmLoad = (ui32SysClock / 2U) / PWM_FREQ;
    // Config PWM generator
    MAP_PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_STOP);
    // Enable PWM triger output
    PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_1, PWM_TR_CNT_ZERO);
    // Set PWM period
    MAP_PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,pwmLoad);
    // Set PWM pulse width
    ui32Width = (uint32_t)(pwmLoad * fDutyCycle);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Width);
    // Enable PWM output
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1); 
}

// SPI
void display_spi_config(void){
    // Enabling and wait for QSSI module and ports F and E to be ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    // Configure pins for QSSI3
    // {PF3:Clk,PE0:CS,PF1:TX,PD0:RX,PE2:rst,PE1:C/D}
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1 | GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    MAP_GPIOPinConfigure(GPIO_PF3_SSI3CLK);
    MAP_GPIOPinConfigure(GPIO_PF1_SSI3XDAT0);
    MAP_GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3 );
    display_enable();
    // Config QSSI2 module, master mode, 16bits len.
    MAP_SSIDisable(SSI3_BASE);
    SSIClockSourceSet(SSI3_BASE, SSI_CLOCK_SYSTEM);
    MAP_SSIConfigSetExpClk(SSI3_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,30000000, 16);
    MAP_SSIEnable(SSI3_BASE);
}

void display_spi_data_len(uint32_t len){
    MAP_SSIDisable(SSI3_BASE);
    MAP_SSIConfigSetExpClk(SSI3_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,30000000, len);
    MAP_SSIEnable(SSI3_BASE);
}

// uDMA
void display_dma_config(void){
    // Enable uDMA peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA));
    // Enable uDMA controller
    MAP_uDMAEnable();
    // Set base address of control table
    MAP_uDMAControlBaseSet(uDMAControlTable);
    // Route SSI3 TX onto its uDMA channel and let the SSI raise TX DMA requests.
    MAP_uDMAChannelAssign(UDMA_CH15_SSI3TX);
    MAP_SSIDMAEnable(SSI3_BASE, SSI_DMA_TX);
    // Dissable DMA channel before config it
    MAP_uDMAChannelDisable(UDMA_CH15_SSI3TX);
    MAP_uDMAChannelAttributeDisable(UDMA_CH15_SSI3TX, UDMA_ATTR_ALL);
    // Configure channel
    MAP_uDMAChannelControlSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
        UDMA_SIZE_16 |        // 16-bit data
        UDMA_SRC_INC_16 |     // increment source
        UDMA_DST_INC_NONE |   // SSI FIFO
        UDMA_ARB_8);          // burst of 8 words
}

void display_snd_dma_buffer(uint16_t* buffer, uint32_t bufferLen){
    // Disable channel before setup
    MAP_uDMAChannelDisable(UDMA_CH15_SSI3TX);
    // Configure transfer
    MAP_uDMAChannelTransferSet(
        UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
        buffer,
        (void *)(SSI3_BASE + SSI_O_DR),
        bufferLen
    );
    // Start transfer
    MAP_uDMAChannelEnable(UDMA_CH15_SSI3TX);
}

// Touch controller
void touch_init(void){
    touch_spi_config();
}

void touch_enable(void){
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x00);
}

void touch_disable(void){
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0);
}

void touch_int_handler(void){
    // Read interrupt status from port L.
    uint32_t int_status = MAP_GPIOIntStatus(GPIO_PORTL_BASE, true);
    // Clear interrupts immediately
    MAP_GPIOIntClear(GPIO_PORTL_BASE, int_status);
    // Set touch flag
    if(int_status & GPIO_PIN_2){// Touch pressed
        touch_asserted = 0x01;
        return;
    }
}

uint8_t touch_pressed(void){
    // Read the latched event and clear it, same pattern as pressed_button():
    // one touch interrupt produces exactly one "pressed" report.
    uint8_t pressed = touch_asserted;
    touch_asserted = 0x00;
    return pressed;
}

uint16_t touch_get_x(void){
    return touch_x;
}

uint16_t touch_get_y(void){
    return touch_y;
}

void touch_request_coords(void){
    uint32_t flush;
    uint16_t raw_x, raw_y;

    // Reading the controller makes PENIRQ pulse, which would otherwise queue a
    // spurious touch interrupt and keep touch_asserted stuck set. Mask PL2 for
    // the duration of the transfer.
    MAP_GPIOIntDisable(GPIO_PORTL_BASE, GPIO_PIN_2);

    touch_enable();                                       // CS low
    while(MAP_SSIDataGetNonBlocking(SSI2_BASE, &flush)){}   // drop stale RX data

    raw_x = touch_read_raw(XPT2046_CMD_X);
    raw_y = touch_read_raw(XPT2046_CMD_Y);

    while(MAP_SSIBusy(SSI2_BASE)){}                         // let the last frame finish
    touch_disable();                                      // CS high

    // Discard the glitch the read produced, then re-arm the touch interrupt.
    MAP_GPIOIntClear(GPIO_PORTL_BASE, GPIO_PIN_2);
    MAP_GPIOIntEnable(GPIO_PORTL_BASE, GPIO_PIN_2);

    // Portrait-flipped panel: both axes run opposite to the raw ADC sweep, so
    // invert each mapped result. Adjust the *_MIN/*_MAX limits during
    // calibration; swap the channel->axis assignment if the panel is mounted
    // rotated 90 degrees.
    touch_x = (ILI9341_WIDTH - 1) - touch_scale(raw_x, XPT2046_X_MIN, XPT2046_X_MAX, ILI9341_WIDTH - 1);
    touch_y = (ILI9341_HEIGHT - 1) - touch_scale(raw_y, XPT2046_Y_MIN, XPT2046_Y_MAX, ILI9341_HEIGHT - 1);

}

// Clock one channel: a 16-bit command frame followed by a 16-bit frame that
// shifts out the BUSY bit + the 12 data bits. The command sits in the low byte
// so the leading zero byte acts as the start-bit gap; on the next frame the
// result lands left-aligned, so (rx >> 3) drops BUSY and the 3 trailing bits.
// Returns the averaged 0..4095 sample. CS is owned by the caller so several
// channels can share a single assertion.
static uint16_t touch_read_raw(uint8_t cmd){
    uint32_t rx;
    uint32_t acc = 0;
    uint8_t i;
    for(i = 0; i < XPT2046_SAMPLES; i++){
        MAP_SSIDataPut(SSI2_BASE, (uint16_t)cmd);   // send command
        MAP_SSIDataGet(SSI2_BASE, &rx);             // discard command echo
        MAP_SSIDataPut(SSI2_BASE, 0x0000);          // clock out the result
        MAP_SSIDataGet(SSI2_BASE, &rx);
        acc += (rx >> 3) & 0x0FFF;
    }
    return (uint16_t)(acc / XPT2046_SAMPLES);
}

// Linear map of a raw reading onto [0, out_max], clamped at the edges.
static uint16_t touch_scale(uint16_t raw, uint16_t in_min, uint16_t in_max, uint16_t out_max){
    if(raw <= in_min){ return 0; }
    if(raw >= in_max){ return out_max; }
    return (uint16_t)(((uint32_t)(raw - in_min) * out_max) / (in_max - in_min));
}

void touch_spi_config(void){
     // Enabling and wait for QSSI module and ports D and L to be ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));
    // Configure pins for QSSI2
    // {PD3:Clk,PL0:CS,PD1:MOSI,PD0:MISO,PL1:rst,PL2:int}
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_1 | GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_0 | GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTL_BASE,GPIO_PIN_2);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_0);
    MAP_GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    MAP_GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
    MAP_GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
    MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 );
    // Enable internal pull-ups for touch interrupt //
    MAP_GPIOPadConfigSet(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    // Disable interrupts during setup
    MAP_GPIOIntDisable(GPIO_PORTL_BASE, GPIO_PIN_2);
    MAP_GPIOIntClear(GPIO_PORTL_BASE, GPIO_PIN_2);
    // Configure interrupt type: falling edge
    MAP_GPIOIntTypeSet(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_FALLING_EDGE);
    // Enable GPIO interrupt (NVIC)
    MAP_IntEnable(INT_GPIOL);
    // Enable GPIO pin interrupts
    MAP_GPIOIntEnable(GPIO_PORTL_BASE, GPIO_PIN_2);
    // Config QSSI2 module, master mode, 16bits len.
    MAP_SSIDisable(SSI2_BASE);
    SSIClockSourceSet(SSI2_BASE, SSI_CLOCK_SYSTEM);
    MAP_SSIConfigSetExpClk(SSI2_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,2000000, 16);
    MAP_SSIEnable(SSI2_BASE);
}


