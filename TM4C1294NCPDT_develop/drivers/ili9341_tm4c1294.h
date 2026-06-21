//*****************************************************************************
// 2.4_tft_display, ILI9341_driver, tm4c1294_spi2_uDMA
//*****************************************************************************
#ifndef ILI9341_TM4C1294_H
#define ILI9341_TM4C1294_H
// Common used libraries.
#include <stdint.h>
#include <stdbool.h>
//*****************************************************************************
// //***** Definitions *****//
//*****************************************************************************
//***** RGB565 basic color format *****//
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

#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320
#define ILI9341_LINES_PER_BUFFER 4
#define ILI9341_CHUNKS_PER_FULL_SCREEN 80
#define FONT_WIDTH   8
#define FONT_HEIGHT 16
#define FONT_FIRST_CHAR 0x20
#define FONT_LAST_CHAR 0x7E
#define FONT_FALLBACK    '?'

// ClockFreq variable
extern volatile uint32_t systemClkFreq;

// Public functions //
void ili9341_reset(void);
void ili9341_send_command(uint8_t cmd);
void ili9341_send_data(uint8_t data);
void ili9341_enable(void);
void ili9341_disable(void);
void ili9341_cmd_mode(void);
void ili9341_data_mode(void);
void ili9341_init(void);
void ili9341_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1);
void ili9341_fill_screen(uint16_t color);
void ili9341_draw_char(uint16_t x, uint16_t y,char c,uint16_t fg, uint16_t bg);
void ili9341_print_string(uint16_t x, uint16_t y,const char *str,uint16_t fg,uint16_t bg);
void ili9341_print_int(uint16_t x, uint16_t y,int32_t num,uint16_t color, uint16_t bg);
void ili9341_print_float(uint16_t x, uint16_t y,float num, uint8_t decimals,uint16_t color, uint16_t bg);
// uDMA //
void uDMA_spi2_config(void);
void uDMA_spi2_send_buffer(uint16_t* dataBuffer, uint32_t bufferLen);
// spi2 FOR DISPLAY //
void spi2_config(void);
void spi2_data_len(uint32_t len);

//*****************************************************************************
// //***** Backlight brightness control (ADC + PWM) *****//
//*****************************************************************************
// The backlight is driven by M0PWM2 (PF2); its duty cycle sets the brightness.
// A control voltage on AIN0 (PE3) is sampled by ADC0 sequencer 3, whose ISR
// maps the 0..4095 reading onto the PWM duty cycle.
//
// ADC counts -> volts (VDDA = 3.3 V reference, 12-bit result).
#define ADC_SCALE (3.3f / 4095.0f)

// Brightness-control state (defined in ili9341_tm4c1294.c).
extern volatile uint32_t adc0Ssq3Value; // Latest raw ADC0 SS3 sample (0..4095)
extern volatile uint8_t adc_ready;   // Set by the ADC ISR; cleared by consumer
extern volatile float fDutyCycle;    // Backlight duty cycle (0.0..1.0)

// Setup helpers — call after systemClkFreq has been set.
void adc0ssq3_config(void);          // Configure ADC0 SS3 (AIN0/PE3)
void adc0ssq3_handler(void);         // ADC0 SS3 ISR (placed in the vector table)
void pwm0_config(void);              // Configure the M0PWM2/PF2 backlight output

// Note: intToStr()/floatToStr() are internal helpers with static linkage,
// defined privately in ili9341_tm4c1294.c (not part of the public API).

#endif // ILI9341_TM4C1294_H

