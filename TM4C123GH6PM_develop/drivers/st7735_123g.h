//*****************************************************************************
// 1.8inch_tft_display (ST7735, 128x160), tm4c123G, spi3_uDMA
//
// Pins:  PD0 = SSI3CLK, PD3 = SSI3TX, PD1 = CS, PD2 = DC, PE1 = RST
//*****************************************************************************
#ifndef ST7735_TM4C123G_H_
#define ST7735_TM4C123G_H_
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common used libraries
#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// DEFINES
//*****************************************************************************
// ST7735 commands
#define ST7735_SWRESET  0x01
#define ST7735_SLPOUT   0x11
#define ST7735_NORON    0x13
#define ST7735_INVOFF   0x20
#define ST7735_DISPON   0x29
#define ST7735_CASET    0x2A
#define ST7735_RASET    0x2B
#define ST7735_RAMWR    0x2C
#define ST7735_MADCTL   0x36
#define ST7735_COLMOD   0x3A
#define ST7735_FRMCTR1  0xB1
#define ST7735_FRMCTR2  0xB2
#define ST7735_FRMCTR3  0xB3
#define ST7735_INVCTR   0xB4
#define ST7735_PWCTR1   0xC0
#define ST7735_PWCTR2   0xC1
#define ST7735_PWCTR3   0xC2
#define ST7735_PWCTR4   0xC3
#define ST7735_PWCTR5   0xC4
#define ST7735_VMCTR1   0xC5
#define ST7735_GMCTRP1  0xE0
#define ST7735_GMCTRN1  0xE1

// RGB565 basic color format (same values as the 2.4inch driver, so including
// both headers in one file is a harmless identical redefinition)
#define BLACK       0x0000
#define NAVY        0x000F
#define DARKGREEN   0x03E0
#define DARKCYAN    0x03EF
#define MAROON      0x7800
#define PURPLE      0x780F
#define OLIVE       0x7BE0
#define LIGHTGREY   0xC618
#define DARKGREY    0x7BEF
#define BLUE        0x001F
#define GREEN       0x07E0
#define CYAN        0x07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define ORANGE      0xFD20
#define GREENYELLOW 0xAFE5
#define PINK        0xFC18

// Display features
// Portrait orientation (panel native 128x160; MADCTL MV bit left clear).
#define ST7735_WIDTH   128
#define ST7735_HEIGHT  160

//*****************************************************************************
// Functions declaration
//*****************************************************************************
// Peripheral config functions
// st7735_gpio_cnfg() must be called once before st7735_init(); it only sets up
// the pins. The SPI/DMA helpers below are called by st7735_init() and expect
// the system clock it was given.
void st7735_gpio_cnfg(void);
void st7735_spi_cnfg(void);
void st7735_spi_len(uint8_t len);
void st7735_dma_cnfg(void);
void st7735_dma_snd_buffer(uint16_t* buffer, uint32_t bufferLen);
// Display functions
// st7735_init() brings up SSI3, resets and configures the panel and arms the
// uDMA channel; it may be called again to re-initialise the display.
// ui32SysClock is the system clock in Hz, i.e. SysCtlClockGet(). Also calls
// delay_init(), so the delay driver needs no separate setup.
void st7735_init(uint32_t ui32SysClock);
void st7735_enable(void);
void st7735_disable(void);
void st7735_rst(void);
void st7735_snd_dt(uint8_t data);
void st7735_snd_cmd(uint8_t cmd);
void st7735_st_wndw(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void st7735_fll_scrn(uint16_t color);
void st7735_drw_chr(uint8_t x, uint8_t y, char c, uint16_t color, uint16_t bg);
void st7735_prtn_str(uint8_t x, uint8_t y, const char* str, uint16_t color, uint16_t bg);
void st7735_prtn_int(uint8_t x, uint8_t y, int32_t value, uint16_t color, uint16_t bg);
void st7735_prtn_float(uint8_t x, uint8_t y, float value, uint8_t decimals, uint16_t color, uint16_t bg);

// Note: the SSI3 clock frequency, the 5x7 font table and the uDMA control
// table are private to st7735_123g.c (not part of the public API).

#endif // ST7735_TM4C123G_H_
