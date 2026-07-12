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
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"

//*****************************************************************************
// DEFINES
//*****************************************************************************
// User switches
#define SW1 GPIO_PIN_0
#define SW2 GPIO_PIN_1
// User leds
#define LED0 GPIO_PIN_0
#define LED1 GPIO_PIN_1
#define LED2 GPIO_PIN_0
#define LED3 GPIO_PIN_4

//*****************************************************************************
// FUNCTIONS DECLARATIONS
//*****************************************************************************
// Buttons
void config_buttons(void);
void int_buttons_handler(void);
uint8_t pressed_button(void);
// Leds
void config_leds(void);
