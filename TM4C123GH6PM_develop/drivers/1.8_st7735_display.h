// Header file to handle a 1.8_st7735_spi_display

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


// SPI3-uDMA
void spi3_config(void);
void spi3_len_config(uint16_t bits);

// uDMA
void uDMA_spi3_config(void);
void uDMA_spi3_send_buffer(uint16_t* dataBuffer, uint32_t count);
void uDMA_spi3_int_handler(void);

// Display
void st7735_init();
void st7735_send_command(uint8_t cmd);
void st7735_send_data(uint8_t data);
void st7735_reset(void);
void st7735_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1);
void st7735_fill_screen(uint16_t color);
void st7735_draw_char(uint16_t x,uint16_t y,char c,uint16_t fg,uint16_t bg);
void st7735_print_string(uint16_t x,uint16_t y,const char *str,uint16_t fg,uint16_t bg);
void int_to_str(int32_t value, char *buf);
void float_to_str(float value, char *buf, uint8_t decimals);
void st7735_print_int(uint16_t x, uint16_t y,int32_t value,uint16_t color, uint16_t bg);
void st7735_print_float(uint16_t x, uint16_t y,float value, uint8_t decimals,uint16_t color, uint16_t bg);
