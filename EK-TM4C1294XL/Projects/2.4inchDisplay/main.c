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
    display_main_screen();
    
    // Loop Forever
    while(1){
        // Read the latched event ONCE: button_pressed() clears it on read,
        // so a second call would see 0 and lose the event.
        uint8_t pressed = pressed_button();
        if(pressed & SW1){ // SW1: run interactive touch calibration
            touch_calibration();
            display_main_screen();
        }
        if(pressed & SW2){ // SW2 pressed
        }
        // Touch event latched by the PENIRQ ISR; touch_pressed() clears it on
        // read, so call it once per loop just like pressed_button().
        if(touch_pressed()){
            // Read the XPT2046 and map raw ADC values to screen pixels. A
            // zero return means the pen was already up (release bounce), so
            // the sample is garbage and must not be printed.
            if(touch_request_coords()){
                // Blank the old digits first so a shorter number (e.g. 45
                // after 239) does not leave stale characters behind.
                display_print_string(64, 64, "   ", GREEN, BLACK);
                display_print_int(64, 64, touch_get_x(), GREEN, BLACK);
                display_print_string(160, 64, "   ", GREEN, BLACK);
                display_print_int(160, 64, touch_get_y(), GREEN, BLACK);
            }
        }
    }
}
