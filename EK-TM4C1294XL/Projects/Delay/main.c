//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common used libraries
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header.
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
// Peripheral features which lives in separated files
#include "drivers/delay.h"

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
    delay_init(ui32SysClock);

    // Enable and wait for PORTF to be ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Configure PF4
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_4);
    // Turn LEDs on initially
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_4, 0x00);

    // Loop Forever
    while(1){
        // Turn on and off a led per second
        MAP_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_PIN_4);
        delay_ms(1000);
        MAP_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_4,0x00);
        delay_ms(1000);
    }
}
