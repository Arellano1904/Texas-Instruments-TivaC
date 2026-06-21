//*****************************************************************************
// Boost converter.
//*****************************************************************************

//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common used libraries.
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header.
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
// Own driver libraries. ADC/PWM backlight-brightness control lives here too.
#include "drivers/ili9341_tm4c1294.h"

//*****************************************************************************
// Definitions.
//*****************************************************************************
#define SYSCLK_HZ 120000000
//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
//*****************************************************************************
// Global variables.
//*****************************************************************************
volatile uint32_t systemClkFreq;

//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void){
    // Run from the PLL at 120 MHz.
    systemClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), SYSCLK_HZ);
    // Floating point unit enable
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // Display init //
    ili9341_init();
    ili9341_fill_screen(BLACK);
    ili9341_print_string(0, 0, "TivaC: EK-TM4C1294XL", RED, BLACK);
    ili9341_print_string(0, 16, "2.4SpiDisplay:ILI9341-240x320", RED, BLACK);

    // Global interrupt enable
    MAP_IntMasterEnable();
    // Loop Forever
    while(1){
    }
}
