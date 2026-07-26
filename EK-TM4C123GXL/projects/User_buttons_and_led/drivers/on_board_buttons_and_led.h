//*****************************************************************************
// Header file to use on board buttons and RGB led on EK-TM4C123GXL
//*****************************************************************************
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common use libraries
#include <stdint.h>
#include <stdbool.h>
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions. 
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
// The inc folder contains the device header files for each TM4C device as well as the hardware header files.
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
//*****************************************************************************
// DEFINES
//*****************************************************************************
// User switches
#define SW1 GPIO_PIN_4
#define SW2 GPIO_PIN_0
// User leds
#define LEDR GPIO_PIN_1
#define LEDB GPIO_PIN_2
#define LEDG GPIO_PIN_3

//*****************************************************************************
// Function declartions
//*****************************************************************************
// Buttons
void config_buttons(void);
void int_buttons_handler(void);
uint8_t pressed_button(void);
// RGB Led
void config_rgb_led(void);

