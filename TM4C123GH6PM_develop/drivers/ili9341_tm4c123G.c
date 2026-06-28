//*****************************************************************************
// 2.4_tft_display, ILI9341_driver, tm4c1294_spi3_uDMA
//*****************************************************************************
/* Pin Connections  
Display ----- EK-TM4C123GXL
  Vcc         3.3v
  GND         GND
  CS          PD1
  RESET       PB5
  A0(DC)      PD2
  SDA         PD3
  SCK         PD0
*/ 
// ili9341DisplayDriver
#include "ili9341_tm4c123G.h"
// Common used libraries.
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header.
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "inc/hw_udma.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/udma.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"

// Internal string helpers (private to this translation unit).
static void intToStr(int32_t value, char *buf);
static void floatToStr(float value, char *buf, uint8_t decimals);
// Blocking delays (private; SysTick-based).
static void delay_us(uint32_t us);
static void delay_ms(uint32_t ms);

// Fonts for display
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

// For uDMA use 
#pragma DATA_ALIGN(uDMAControlTable, 1024);
uint8_t uDMAControlTable[1024];
static uint16_t colorBufer[ILI9341_WIDTH * ILI9341_LINES_PER_BUFFER];
static uint16_t g_ui16CharBuf[FONT_WIDTH * FONT_HEIGHT];

//*****************************************************************************
// Blocking delays built on the Cortex-M SysTick down-counter.
//
// SysTick is a dedicated 24-bit core timer, so these delays are clock-accurate
// (paced by systemClkFreq) and consume no general-purpose timer. It is unused
// elsewhere and only runs at boot during display init/reset, so blocking on it
// is safe. Replaces the approximate, hand-tuned MAP_SysCtlDelay() loops.
//*****************************************************************************
static void delay_us(uint32_t us){
    const uint32_t ticksPerUs = MAP_SysCtlClockGet() / 1000000U;  // 80 @ 80 MHz
    const uint32_t maxChunkUs = 100000U;                          // keeps reload < 2^24
    while(us){
        uint32_t chunk = (us > maxChunkUs) ? maxChunkUs : us;
        MAP_SysTickPeriodSet(chunk * ticksPerUs);
        HWREG(NVIC_ST_CURRENT) = 0;   // any write clears the counter and COUNTFLAG
        MAP_SysTickEnable();
        // Spin until the counter reloads (COUNTFLAG, bit 16, asserts).
        while((HWREG(NVIC_ST_CTRL) & NVIC_ST_CTRL_COUNT) == 0){ }
        MAP_SysTickDisable();
        us -= chunk;
    }
}

static void delay_ms(uint32_t ms){
    while(ms--)
        delay_us(1000U);
}

// DISPLAY //
void ili9341_enable(void){
    // CS to low (Enable the Display) //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 0);
}
void ili9341_disable(void){
    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, GPIO_PIN_1);
}
void ili9341_cmd_mode(void){
    // DC to low put the Display  in command mode //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);
}
void ili9341_data_mode(void){
    // DC to low put the Display  in command mode //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

void ili9341_reset(void){
    // RESET pin low: assert reset (datasheet min pulse 10 us; 20 ms is generous).
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
    delay_ms(20);
    // RESET pin high: release reset, then wait for the controller to come up.
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    delay_ms(120);
}

void ili9341_send_command(uint8_t cmd){
    ili9341_enable();
    ili9341_cmd_mode();
    // Send the command through SSI3
    MAP_SSIDataPut(SSI3_BASE, cmd);
    // Wait for data to be transferred
    while(MAP_SSIBusy(SSI3_BASE));
    ili9341_disable();
}

void ili9341_send_data(uint8_t data){
    ili9341_enable();
    ili9341_data_mode();
    // Send the data to SSI3
    MAP_SSIDataPut(SSI3_BASE, data);
    // Wait for data to be transferred
    while(MAP_SSIBusy(SSI3_BASE));
    ili9341_disable();
}

//*****************************************************************************
// Power-on initialization sequence.
//
// Data-driven table: each row is { command, parameter count, parameters,
// post-command delay (ms) }. The power/VCOM/gamma registers are the
// manufacturer-recommended ILI9341 values; together with the 119 Hz frame
// rate (FRMCTR1) and the existing 30 MHz SPI + uDMA path they give the best
// image quality and the fastest refresh the panel supports.
//*****************************************************************************
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

void ili9341_init(void){
    // Peripherals: SPI link, uDMA pixel pump, and the ADC/PWM backlight control.
    spi3_config();
    uDMA_spi3_config();
    // adc0ssq3_config();
    // pwm0_config();

    // Hardware reset (reloads factory defaults; includes the required delays).
    ili9341_reset();

    // Commands and their parameters are single bytes.
    spi3_data_len(8);

    uint32_t i;
    uint8_t  a;
    for (i = 0; i < sizeof(ili9341_init_seq) / sizeof(ili9341_init_seq[0]); i++){
        const ILI9341_InitCmd *c = &ili9341_init_seq[i];
        ili9341_send_command(c->cmd);
        for (a = 0; a < c->numArgs; a++)
            ili9341_send_data(c->args[a]);
        if (c->delayMs)
            delay_ms(c->delayMs);
    }

    // Switch to 16-bit frames for fast pixel streaming via uDMA.
    spi3_data_len(16);
}

void ili9341_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1){
    // Config spi3 to send command and data configuration
    spi3_data_len(8);
    // Column Address Set (CASET)
    ili9341_send_command(ILI9341_CASET);
    ili9341_send_data(x0 >> 8);     // X start high
    ili9341_send_data(x0 & 0xFF);   // X start low
    ili9341_send_data(x1 >> 8);     // X end high
    ili9341_send_data(x1 & 0xFF);   // X end low

    // Page Address Set (PASET)
    ili9341_send_command(ILI9341_PASET);
    ili9341_send_data(y0 >> 8);     // Y start high
    ili9341_send_data(y0 & 0xFF);   // Y start low
    ili9341_send_data(y1 >> 8);     // Y end high
    ili9341_send_data(y1 & 0xFF);   // Y end low

    // Write to RAM
    ili9341_send_command(ILI9341_RAMWR);
    // Config spi3 to send pixels data
    spi3_data_len(16);
    
}

void ili9341_fill_screen(uint16_t color){
    uint16_t  i;
    // Fill a chunk color buffer
    for(i=0;i<ILI9341_WIDTH*ILI9341_LINES_PER_BUFFER;i++)
        colorBufer[i] = color;
    
    ili9341_set_window(0, 0, ILI9341_WIDTH - 1, ILI9341_HEIGHT - 1);
    ili9341_enable();
    ili9341_data_mode();

    for (i = 0; i < ILI9341_HEIGHT; i+=ILI9341_LINES_PER_BUFFER){
        /*
         * Wait only for DMA done — NOT for SSIBusy.
         * The FIFO keeps clocking bits out in the background while we poll.
         * The next DMA burst starts feeding the FIFO as soon as it has room,
         * with zero dead time on the SPI bus.
         */
        while (MAP_uDMAChannelSizeGet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT));

        uDMA_spi3_send_buffer(colorBufer, ILI9341_WIDTH * ILI9341_LINES_PER_BUFFER);
    }

    /* Wait for the shift register to drain only once, at the very end */
    while (SSIBusy(SSI3_BASE));
}

void ili9341_draw_char(uint16_t x, uint16_t y,char ch,uint16_t fgColor, uint16_t bgColor){
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
    ili9341_set_window(x,y,x + FONT_WIDTH - 1, y + FONT_HEIGHT - 1);

    ili9341_enable();      // CS low
    ili9341_data_mode();   // DC high — pixel stream

    // Kick off DMA: 128 pixels × 16-bit = 256 bytes
    uDMA_spi3_send_buffer(g_ui16CharBuf, FONT_WIDTH * FONT_HEIGHT);

    // Must wait for BOTH DMA done AND shift register empty before
    // the next ili9341_set_window() call. set_window sends SPI commands
    // and if SSI is still shifting pixels you corrupt the pixel stream.
    while (MAP_uDMAChannelSizeGet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT));
    while (SSIBusy(SSI3_BASE));

    ili9341_disable();     // CS high
    
}

void ili9341_print_string(uint16_t x, uint16_t y,const char *str,uint16_t fgColor,uint16_t bgColor){
    uint16_t curX = x;
    uint16_t curY = y;

    while (*str != '\0')
    {
        char ch = *str++;

        // ── Newline ───────────────────────────────────────────────────────────
        if (ch == '\n')
        {
            curX  = x;          // Return to left margin (not column 0 — respects x)
            curY += FONT_HEIGHT;     // Drop one text row (16 pixels)
            if (curY + FONT_HEIGHT > ILI9341_HEIGHT) return;   // Off bottom → stop
            continue;
        }

        if (ch == '\r') continue;   // Ignore bare carriage return

        // ── Auto word-wrap ───────────────────────────────────────────────────
        // Don't let a character start where it would run past the right edge
        if (curX + FONT_WIDTH > ILI9341_WIDTH)
        {
            curX  = x;
            curY += FONT_HEIGHT;
            if (curY + FONT_HEIGHT > ILI9341_HEIGHT) return;
        }

        ili9341_draw_char(curX, curY, ch, fgColor, bgColor);
        curX += FONT_WIDTH;    // Advance cursor one glyph width
    }
} 

void ili9341_print_int(uint16_t x, uint16_t y,int32_t num,uint16_t color, uint16_t bg){
    char buf[12];   // "-2147483648" + '\0' = 12 bytes
    intToStr(num, buf);
    ili9341_print_string(x,y,buf,color,bg);
}

void ili9341_print_float(uint16_t x, uint16_t y,float num, uint8_t decimals,uint16_t color, uint16_t bg){
    // '-' + up to 10 integer digits + '.' + decimals + '\0'.
    // Clamp decimals so the result always fits the buffer.
    char buf[24];
    if (decimals > 9) decimals = 9;
    floatToStr(num, buf,decimals);
    ili9341_print_string(x,y,buf,color,bg);
}

// spi3 for display //
void spi3_config(void){
    // Enable SSI3, PORT_D and PORT_B //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // Configure GPIO Port D pins 0 and 3 to be used as SSI
    MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_3);

    // Configure PD0, PD3 for SSI3 (CLK, TX) //
    MAP_GPIOPinConfigure(GPIO_PD0_SSI3CLK); 
    MAP_GPIOPinConfigure(GPIO_PD3_SSI3TX);

    // Configure ChipSelect (CS-PD1) and DataComand (DC-PD2) pins  //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2);

    // Configure RESET pin (PB5) //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,  GPIO_PIN_5);

    // Configure the SSI3 as SPI MODE0, 16-bit frames, 30 MHz bit clock //
    MAP_SSIConfigSetExpClk(SSI3_BASE,MAP_SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,30000000,16);
    // Enabling the SSI3 module //
    MAP_SSIEnable(SSI3_BASE);
}

void spi3_data_len(uint32_t len){
    MAP_SSIDisable(SSI3_BASE);
    MAP_SSIConfigSetExpClk(SSI3_BASE, systemClkFreq, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,30000000, len);
    MAP_SSIEnable(SSI3_BASE);
}

// uDMA //
void uDMA_spi3_config(void){
    // Enable uDMA peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA));

    // Enable uDMA controller
    MAP_uDMAEnable();

    // Set base address of control table
    MAP_uDMAControlBaseSet(uDMAControlTable);

    // Enable uDMA on SSI3 TX
    MAP_SSIDMAEnable(SSI3_BASE, SSI_DMA_TX);

    // Optional but safe
    MAP_uDMAChannelAssign(UDMA_CH15_SSI3TX);

    // Disable channel before configuration
    MAP_uDMAChannelDisable(UDMA_CH15_SSI3TX);

    // Configure channel
    MAP_uDMAChannelControlSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
        UDMA_SIZE_16 |        // 16-bit data (one RGB565 pixel per item)
        UDMA_SRC_INC_16 |     // walk through the source buffer
        UDMA_DST_INC_NONE |   // SSI3 data register stays put
        UDMA_ARB_8);          // re-arbitrate every 8 items (SSI FIFO depth)

    // Transfers are polled to completion in uDMA_spi3_send_buffer(), so the
    // uDMA completion interrupt is left disabled on purpose (no ISR exists).
}

void uDMA_spi3_send_buffer(uint16_t* dataBuffer, uint32_t count){
    // CS low to enable the Display and DC high to data mode //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1 | GPIO_PIN_2, 0x04);
    // Disable channel before setup
    MAP_uDMAChannelDisable(UDMA_CH15_SSI3TX);

    // Configure transfer
    MAP_uDMAChannelTransferSet(
       UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
        dataBuffer,
        (void *)(SSI3_BASE + SSI_O_DR),
        count
    );

    // Start transfer
    MAP_uDMAChannelEnable(UDMA_CH15_SSI3TX);
    // Wait for completion
    while(MAP_uDMAChannelIsEnabled(UDMA_CH15_SSI3TX));
    // Wait for SSI to finish shifting last word
    while(MAP_SSIBusy(SSI3_BASE));

    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

// Funciones de apoyo
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

// //*****************************************************************************
// // Backlight brightness control.
// //
// // The panel backlight is driven by M0PWM2 (PF2); its duty cycle sets the
// // brightness. A control voltage on AIN0 (PE3) is sampled by ADC0 sequencer 3,
// // and the ADC interrupt maps the reading (0..4095) onto the PWM duty cycle.
// // ADC0 SS3 is triggered by PWM generator 1, so conversions are paced by the
// // backlight PWM itself.
// //*****************************************************************************
// #define PWM_FREQ 500U   // Backlight PWM frequency (Hz)

// // Latest raw ADC0 SS3 sample (0..4095). Written by the ISR, read by the app.
// volatile uint32_t adc0Ssq3Value = 0x0000;
// // Set to 1 by the ADC ISR when a new sample is ready; cleared by the consumer.
// volatile uint8_t adc_ready = 0x00;
// // Backlight duty cycle, 0.0..1.0. Updated from the ADC reading in the ISR.
// volatile float fDutyCycle = 0.50f;   // 50%
// // PWM period in clock ticks, computed at runtime from the system clock.
// static uint32_t pwmLoad = 0x0000;
// // PWM pulse width in clock ticks (derived from fDutyCycle).
// static uint32_t ui32Width = 0x0000;

// void adc0ssq3_config(void){
//     // Enable the ADC module and related peripheral
//     MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
//     // Wait for the ADC0 module to be ready
//     while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
//     MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//     while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
//     // Disable digital function on the pin
//     MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
//     // Config ADC clock
//     ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL,12);
//     // Use VDDA (3.3 V)
//     MAP_ADCReferenceSet(ADC0_BASE, ADC_REF_INT);
//     // Configure ADC oversampling
//     MAP_ADCHardwareOversampleConfigure(ADC0_BASE, 0);
//     // Disable sequencer 3 during config
//     MAP_ADCSequenceDisable(ADC0_BASE, 3);
//     // Enable the first sample sequencer to capture the value of channel 0 when the processor trigger occurs
//     MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PWM_MOD0 | ADC_TRIGGER_PWM1, 0);
//     // Configure sequencer steps
//     MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
//     // Clear ADC0 Sequencer 3 interrupt
//     MAP_ADCIntClear(ADC0_BASE, 3);
//     // Enable ADC0 Sequencer 3 interrupt
//     MAP_ADCIntEnable(ADC0_BASE, 3);
//     // Enable interrupt in NVIC
//     MAP_IntEnable(INT_ADC0SS3);
//     // Enable ADC sequencer
//     MAP_ADCSequenceEnable(ADC0_BASE, 3);

// }
// // ADC0 Sequencer 3 interrupt handler
// void adc0ssq3_handler(void){
//     uint32_t sample = 0;
//     // ALWAYS clear first, before reading
//     MAP_ADCIntClear(ADC0_BASE, 3);
//     // Read ADC result into a non-volatile local, then publish it (avoids
//     // passing a volatile pointer to the DriverLib API).
//     MAP_ADCSequenceDataGet(ADC0_BASE, 3, &sample);
//     adc0Ssq3Value = sample;
//     // Update PWM duty cycle based on ADC reading (0-4095 -> 0-100%)
//     fDutyCycle = (float)sample / 4095.0f;
//     ui32Width  = (uint32_t)(pwmLoad * fDutyCycle);
//     // Clamp: PWM hardware requires 1 <= width <= (pwmLoad - 1)
//     // If width == 0 or >= pwmLoad the comparator never fires -> output glitches
//     if(ui32Width < 1)           ui32Width = 1;
//     if(ui32Width > pwmLoad - 1) ui32Width = pwmLoad - 1;
//     MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Width);
//     // Signal main loop that a new ADC value is ready
//     adc_ready = 0x01;
// }

// void pwm0_config(void){
//     // Enabling and waiting for related peripheral
//     MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
//     while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
//     MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
//     while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
//     // Configure pin PM3 as PWM output
//     MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
//     MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
//     // Config PWM module clock
//     MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_2);
//     // PWM period in ticks, derived from the system clock (PWM_SYSCLK_DIV_2 halves it)
//     pwmLoad = (systemClkFreq / 2U) / PWM_FREQ;
//     // Config PWM generator
//     MAP_PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_STOP);
//     // Enable PWM triger output
//     PWMGenIntTrigEnable(PWM0_BASE, PWM_GEN_1, PWM_TR_CNT_ZERO);
//     // Set PWM period
//     MAP_PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,pwmLoad);
//     // Set PWM pulse width
//     ui32Width = (uint32_t)(pwmLoad * fDutyCycle);
//     MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Width);
//     // Enable PWM output
//     MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
//     MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
// }
