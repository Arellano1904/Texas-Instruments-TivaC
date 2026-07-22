//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common used libraries
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header.
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "inc/hw_pwm.h"
#include "inc/hw_ssi.h"
#include "inc/hw_udma.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"

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
#define PINK        0xFC18
// Display features
#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320
#define ILI9341_LINES_PER_BUFFER 4
// Font related definitions
#define FONT_WIDTH   8
#define FONT_HEIGHT 16
#define FONT_FIRST_CHAR 0x20
#define FONT_LAST_CHAR 0x7E
#define FONT_FALLBACK '?'
// ILI9341 command set 
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
// Raw 12-bit ADC values seen at the panel edges. These are only power-on
// defaults: touch_calibration() measures the real per-unit limits at runtime
// and shows them so they can be copied here to persist across resets.
#define XPT2046_X_MIN       350  // 10
#define XPT2046_X_MAX       3850 // 4076
#define XPT2046_Y_MIN       650  // 10
#define XPT2046_Y_MAX       3850 // 4076
// Inset of the calibration crosses from the screen corners, in pixels.
// Must be >= 8 so the '+' glyph cell stays fully on screen.
#define XPT2046_CAL_MARGIN  10
// ADC and PWM for brightness
#define PWM_FREQ 500U   // Backlight PWM frequency (Hz)
//*****************************************************************************
// Functions declaration
//*****************************************************************************
// Public display functions
void display_init(void);
void display_rst(void);
void display_enable(void);
void display_disable(void);
void display_snd_cmd(uint8_t cmd);
void display_snd_data(uint8_t data);
void display_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1);
void display_fill_screen(uint16_t color);
void display_draw_char(uint16_t x, uint16_t y,char c,uint16_t fg, uint16_t bg);
void display_print_string(uint16_t x, uint16_t y,const char *str,uint16_t fg,uint16_t bg);
void display_print_int(uint16_t x, uint16_t y,int32_t num,uint16_t color, uint16_t bg);
void display_print_float(uint16_t x, uint16_t y,float num, uint8_t decimals,uint16_t color, uint16_t bg);
void display_main_screen(void);
// Internal string helpers (private to this translation unit).
static void intToStr(int32_t value, char *buf);
static void floatToStr(float value, char *buf, uint8_t decimals);
// Display brightness controller
// ADC 
void display_adc_config(void);
void display_adc_handler(void);
// PWM
void display_pwm_config(void);
// Display screen controller
// SPI
void display_spi_config(void);
void display_spi_data_len(uint32_t len);
// DMA
void display_dma_config(void);
void display_snd_dma_buffer(uint16_t* buffer, uint32_t bufferLen);
// Display Touch controller
void touch_init(void);
void touch_enable(void);
void touch_disable(void);
// Samples the XPT2046 and updates the cached coords. Returns 1 on a valid
// read, 0 when the pen was up before or lifted during the read (coords kept).
uint8_t touch_request_coords(void);
// Interactive two-point calibration. Takes over the screen, guides the user
// through two crosses and updates the runtime calibration limits. The caller
// must redraw its own screen afterwards.
void touch_calibration(void);
void touch_int_handler(void);
// Touch state accessors: touch_pressed() reads and clears the latched touch
// event; touch_get_x/y() return the last coords from touch_request_coords().
uint8_t touch_pressed(void);
uint16_t touch_get_x(void);
uint16_t touch_get_y(void);
// Raw 12-bit readings from the last valid touch_request_coords(), for
// calibrating the XPT2046_*_MIN/MAX limits against the physical edges.
uint16_t touch_get_raw_x(void);
uint16_t touch_get_raw_y(void);
static uint16_t touch_read_raw(uint8_t cmd);
static uint16_t touch_scale(uint16_t raw, uint16_t in_min, uint16_t in_max, uint16_t out_max);
// Calibration helpers (private to this translation unit): wait for a clean
// pen release, and sample the raw ADC at one on-screen cross.
static void touch_cal_wait_release(void);
static void touch_cal_point(uint16_t sx, uint16_t sy, uint16_t *rx, uint16_t *ry);
// SPI
void touch_spi_config(void);

