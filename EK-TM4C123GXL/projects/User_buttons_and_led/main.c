//*****************************************************************************
// Main file to test user buttons and RGB led
//*****************************************************************************
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common use libraries
#include <stdint.h>
#include <stdbool.h>
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions. 
#include "driverlib/sysctl.h"
// Own drivers
#include "drivers/on_board_buttons_and_led.h"

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line){
}
#endif

//*****************************************************************************
// Main 'C' Language entry point.  Toggle the RGB LED with the on board buttons.
//*****************************************************************************
int main(void){
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Configure the on board buttons (interrupt driven) and the RGB LED.
    config_buttons();
    config_rgb_led();

    // Loop Forever
    while(1){
        uint8_t buttons = pressed_button();
        if(buttons & SW1){      // SW1 toggles the red LED.
            MAP_GPIOPinWrite(GPIO_PORTF_BASE,LEDR,LEDR);
        }
        if(buttons & SW2){      // SW2 toggles the green LED.
            
            MAP_GPIOPinWrite(GPIO_PORTF_BASE,LEDR,0x00);
        }
    }
}
