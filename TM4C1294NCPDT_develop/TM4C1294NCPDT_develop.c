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
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
// Own driver libraries.
#include "drivers/ili9341_tm4c1294.h"

//*****************************************************************************
// Definitions.
//*****************************************************************************

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
    systemClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);
    // Display init.
    ili9341_init();
    ili9341_fill_screen(BLACK);
    // Loop Forever
    while(1){}
}
