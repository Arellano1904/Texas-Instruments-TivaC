//*****************************************************************************
// 2.4_tft_display, ILI9341_driver, tm4c1294_spi0_uDMA
//*****************************************************************************
// Common used libraries.
#include <stdint.h>
#include <stdbool.h>
//*****************************************************************************
// //***** Definitions *****//
//*****************************************************************************
//***** Colores b�sicos en formato RGB565 *****//
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
#define PINK        0xF81F

#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320
#define FONT_WIDTH   8
#define FONT_HEIGHT 16
#define FONT_FIRST_CHAR 0x20
#define FONT_LAST_CHAR 0x7E



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
void uDMA_spi0_config(void);
void uDMA_spi0_send_buffer(uint16_t* dataBuffer, uint32_t bufferLen);
static inline void fill_buf_u32(uint16_t *buf, uint16_t color, uint32_t pixels);
// SPI0 FOR DISPLAY //
void spi0_config(void);
void spi0_data_len(uint32_t len);
// Funciones de apoyo
static void intToStr(int32_t value, char *buf);
static void floatToStr(float value, char *buf, uint8_t decimals);

