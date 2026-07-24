//*****************************************************************************
// Main develop file for TM4C123GH6PM uC on EK-TM4C123GXL board on register
// level approach.
//*****************************************************************************
//*****************************************************************************
// LIBRARIES 
//*****************************************************************************
// Common used libraries
#include <stdint.h>
#include <stdbool.h>
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions. 
#include "driverlib/sysctl.h"  // To config system clock freq with no register levele programming yet.
#include "driverlib/rom_map.h" // To use, if needed, the driverlib functions stored in ROM.
#include "driverlib/fpu.h"     // To use Floating point unit 
// Header file to work with TM4C123GH6PM
#include "inc/tm4c123gh6pm.h"

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line){
}
#endif

//*****************************************************************************
// Global vars.
//*****************************************************************************
uint32_t sysClkFq = 0x00000000;

//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void){
    // Setup the system clock to run at 80Mhz from PLL with crystal reference
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    sysClkFq = MAP_SysCtlClockGet();
    // Floating point unit enabling //
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // Loop Forever
    while(1){
        
    }
}
