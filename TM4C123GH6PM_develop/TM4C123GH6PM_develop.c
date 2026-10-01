//*****************************************************************************
// Main develop file for TM4C123GH6PM
//*****************************************************************************
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common use libraries
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header.
#include "inc/hw_memmap.h"
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
// Own drivers
#include "drivers/on_board_buttons_and_led.h"
#include "drivers/st7735_123g.h"

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line){
}
#endif
//*****************************************************************************
// GLOBALS VARIABLES
//*****************************************************************************
uint32_t SysClkFrq = 0x00000000;

//*****************************************************************************
// Main 'C' Language entry point.  Toggle the RGB LED with the on board buttons.
//*****************************************************************************
int main(void){
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    SysClkFrq = SysCtlClockGet();
    // Floating point unit enabling //
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // Configure the on board buttons (interrupt driven) and the RGB LED.
    config_buttons();
    config_rgb_led();
    // Display
    st7735_gpio_cnfg();


    // Loop Forever
    while(1){
        // Read the latched event ONCE: button_pressed() clears it on read,
        // so a second call would see 0 and lose the event.
        uint8_t buttons = pressed_button();
        if(buttons & SW1){      // SW1 pressed.
            MAP_GPIOPinWrite(GPIO_PORTF_BASE,LEDR,LEDR);
            st7735_init(SysClkFrq);
        }
        if(buttons & SW2){      // SW2 pressed.
            MAP_GPIOPinWrite(GPIO_PORTF_BASE,LEDR,0x00);
            // Black canvas, then some sample strings and numbers on top of it
            st7735_fll_scrn(BLACK);
            st7735_prtn_str(0, 0,  "TM4C123 ST7735", RED, BLACK);
            st7735_prtn_str(0, 8, "DMA pixel push", BLUE, BLACK);
            st7735_prtn_int(0, 16, -12345, GREEN, BLACK);
            st7735_prtn_float(0,24, 3.14159f, 3, PURPLE, BLACK);
        }
    }
}
