//*****************************************************************************
// xpt2046 driver for 2.4_tft_display, ILI9341_driver, tm4c1294_spi2_uDMA
//*****************************************************************************
// Common used libraries.
#include <stdint.h>
#include <stdbool.h>

// ClockFreq variable
extern volatile uint32_t systemClkFreq;

// Last touch position in screen pixels, updated by xpt2046_request_coordinates().
extern volatile uint16_t touch_x;
extern volatile uint16_t touch_y;

//*****************************************************************************
// //***** Functions declarations *****//
//*****************************************************************************
// xpt2046 related functions
void xpt2046_init(void);
void xpt2046_enable(void);
void xpt2046_disable(void);
void xpt2046_request_coordinates(void);
void xpt2046_int_handler(void);
// SPI for xpt2046 driver
void spi2_config(void);

