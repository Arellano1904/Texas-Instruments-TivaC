//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common used libraries
#include <stdint.h>
#include <stdbool.h>
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/sysctl.h"
// Config peripheral functions which lives in separated files
#include "drivers/on_board_buttons_and_leds.h"

//*****************************************************************************
// GLOBAL VARS
//*****************************************************************************

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line){
}
#endif

//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void){
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    uint32_t  ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);
    
    // Config peripheral functions
    config_leds();
    config_buttons();

    // Loop Forever
    while(1){
        // Read the latched event ONCE: button_pressed() clears it on read,
        // so a second call would see 0 and lose the event.
        uint8_t pressed = pressed_button();
        if(pressed & SW1){ // SW1 pressed
            MAP_GPIOPinWrite(GPIO_PORTN_BASE,LED0 | LED1,0x03);
            MAP_GPIOPinWrite(GPIO_PORTF_BASE,LED2| LED3,0x11);
        }
        if(pressed & SW2){ // SW2 pressed
            MAP_GPIOPinWrite(GPIO_PORTN_BASE,LED0 | LED1,0x00);
            MAP_GPIOPinWrite(GPIO_PORTF_BASE,LED2| LED3,0x00);
        }
    }
}
