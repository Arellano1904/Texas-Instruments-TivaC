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
#include "drivers/2.4inch_tft_display.h"

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line){
}
#endif

//*****************************************************************************
// GLOBALS
//*****************************************************************************
// System clock frequency (Hz). Global so peripheral drivers can reference it
// via `extern uint32_t ui32SysClock;` (see drivers/2.4inch_tft_display.c).
uint32_t ui32SysClock;

//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void){
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);
    
    // Config peripheral functions
    config_leds();
    config_buttons();
    display_init();

    // Printing info
    display_fill_screen(BLACK);
    display_print_string(0, 0, "Microcontroller:TM4C1294NCPDT", RED, BLACK);
    display_print_string(0, 16, "Display driver:ILI9341-240x320p", RED, BLACK);
    display_print_string(0, 32, "Touch controller:XPT2046", RED, BLACK);
    
    // Loop Forever
    while(1){
        // Read the latched event ONCE: button_pressed() clears it on read,
        // so a second call would see 0 and lose the event.
        uint8_t pressed = pressed_button();
        if(pressed & SW1){ // SW1 pressed
        }
        if(pressed & SW2){ // SW2 pressed
        }
    }
}
