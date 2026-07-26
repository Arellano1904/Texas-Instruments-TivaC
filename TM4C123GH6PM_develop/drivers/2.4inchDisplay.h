//*****************************************************************************
// 2.4inch_tft_display (ILI9341 + XPT2046 touch), tm4c123G, spi3_uDMA + spi0
//*****************************************************************************
#ifndef DISPLAY_2_4INCH_TM4C123G_H
#define DISPLAY_2_4INCH_TM4C123G_H
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common used libraries
#include <stdint.h>
#include <stdbool.h>

//*****************************************************************************
// DEFINES
//*****************************************************************************
// RGB565 basic color format
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
#define PINK        0xFC18      // Distinct from MAGENTA (0xF81F)

// Display features
// Portrait orientation (panel native 240x320; MADCTL MV bit left clear).
#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320
#define ILI9341_LINES_PER_BUFFER 4
#define ILI9341_CHUNKS_PER_FULL_SCREEN 80   // ILI9341_HEIGHT / ILI9341_LINES_PER_BUFFER

// Font related definitions
#define FONT_WIDTH   8
#define FONT_HEIGHT 16
#define FONT_FIRST_CHAR 0x20
#define FONT_LAST_CHAR 0x7E
#define FONT_FALLBACK    '?'

// ILI9341 command set (subset used by this driver)
#define ILI9341_SWRESET 0x01   // Software reset
#define ILI9341_SLPOUT  0x11   // Sleep out
#define ILI9341_DISPON  0x29   // Display on
#define ILI9341_CASET   0x2A   // Column address set
#define ILI9341_PASET   0x2B   // Page address set
#define ILI9341_RAMWR   0x2C   // Memory write
#define ILI9341_MADCTL  0x36   // Memory access control
#define ILI9341_COLMOD  0x3A   // Pixel format set

// XPT2046 control bytes: start=1, channel select, MODE=0 (12-bit),
// SER/DFR=0 (differential), PD1:PD0=00 (power down between conversions so
// PENIRQ stays enabled).
#define XPT2046_CMD_X       0x90    // X position (A2:A0 = 001)
#define XPT2046_CMD_Y       0xD0    // Y position (A2:A0 = 101)
// Samples averaged per axis to suppress jitter.
#define XPT2046_SAMPLES     8
// Raw 12-bit ADC values seen at the panel edges. Calibrate these per unit.
#define XPT2046_X_MIN       20
#define XPT2046_X_MAX       4076
#define XPT2046_Y_MIN       20
#define XPT2046_Y_MAX       4076
// Active area of the 2.4" panel in the current (portrait) orientation.
#define XPT2046_SCREEN_W    240
#define XPT2046_SCREEN_H    320

// ADC and PWM for brightness
// ADC counts -> volts (VDDA = 3.3 V reference, 12-bit result).
#define ADC_SCALE (3.3f / 4095.0f)

//*****************************************************************************
// GLOBAL VARIABLES
//*****************************************************************************
// ClockFreq variable
extern volatile uint32_t systemClkFreq;

// Last touch position in screen pixels, updated by touch_request_coords().
extern volatile uint16_t touch_x;
extern volatile uint16_t touch_y;

// Brightness-control state (defined in 2.4inchDisplay.c).
// The backlight is driven by M0PWM2 (PF2); its duty cycle sets the brightness.
// A control voltage on AIN0 (PE3) is sampled by ADC0 sequencer 3, whose ISR
// maps the 0..4095 reading onto the PWM duty cycle.
extern volatile uint32_t adc0Ssq3Value; // Latest raw ADC0 SS3 sample (0..4095)
extern volatile uint8_t adc_ready;   // Set by the ADC ISR; cleared by consumer
extern volatile float fDutyCycle;    // Backlight duty cycle (0.0..1.0)

//*****************************************************************************
// Functions declaration
//*****************************************************************************
// Public display functions
void display_init(void);
void display_rst(void);
void display_enable(void);
void display_disable(void);
void display_cmd_mode(void);
void display_data_mode(void);
void display_snd_cmd(uint8_t cmd);
void display_snd_data(uint8_t data);
void display_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1);
void display_fill_screen(uint16_t color);
void display_draw_char(uint16_t x, uint16_t y,char c,uint16_t fg, uint16_t bg);
void display_print_string(uint16_t x, uint16_t y,const char *str,uint16_t fg,uint16_t bg);
void display_print_int(uint16_t x, uint16_t y,int32_t num,uint16_t color, uint16_t bg);
void display_print_float(uint16_t x, uint16_t y,float num, uint8_t decimals,uint16_t color, uint16_t bg);
// Display brightness controller
// ADC — setup helpers, call after systemClkFreq has been set.
void display_adc_config(void);       // Configure ADC0 SS3 (AIN0/PE3)
void display_adc_handler(void);      // ADC0 SS3 ISR (placed in the vector table)
// PWM
void display_pwm_config(void);       // Configure the M0PWM2/PF2 backlight output
// Display screen controller
// SPI (SSI3 for the display)
void display_spi_config(void);
void display_spi_data_len(uint32_t len);
// DMA
void display_dma_config(void);
void display_snd_dma_buffer(uint16_t* buffer, uint32_t bufferLen);
// Display Touch controller
void touch_init(void);
void touch_enable(void);
void touch_disable(void);
void touch_request_coords(void);
void touch_int_handler(void);
// SPI (SSI0 for the touch controller)
void touch_spi_config(void);

// Note: intToStr()/floatToStr() and touch_read_raw()/touch_scale() are
// internal helpers with static linkage, defined privately in 2.4inchDisplay.c
// (not part of the public API).

#endif // DISPLAY_2_4INCH_TM4C123G_H
