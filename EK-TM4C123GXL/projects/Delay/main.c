//*****************************************************************************
// Main develop file for TM4C123GH6PM
//*****************************************************************************
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common use libraries
#include <stdint.h>
#include <stdbool.h>
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions. 
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
// Own drivers
#include "drivers/delay.h"

//*****************************************************************************
// DEFINES
//*****************************************************************************
// User leds
#define LEDR GPIO_PIN_1
#define LEDB GPIO_PIN_2
#define LEDG GPIO_PIN_3

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line){
}
#endif

//*****************************************************************************
// Functions declarations
//*****************************************************************************
void init_rgb_led(void);

//*****************************************************************************
// Main 'C' Language entry point.  Blink among R, G and B on the RGB LED.
//*****************************************************************************
int main(void){
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    // Initialize the delay module with the actual system clock frequency.
    delay_init(MAP_SysCtlClockGet());

    // Floating point unit enabling //
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // Init rgb led to proof delay driver
    init_rgb_led();

    // Loop Forever
    while(1){
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, LEDR | LEDG | LEDB, LEDR);    // Red on
        delay_ms(1000);
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, LEDR | LEDG | LEDB, LEDG);    // Green on
        delay_ms(1000);
        MAP_GPIOPinWrite(GPIO_PORTF_BASE, LEDR | LEDG | LEDB, LEDB);    // Blue on
        delay_ms(1000);
    }
}
//*****************************************************************************
// Functions definitions
//*****************************************************************************

void init_rgb_led(void){
    // Enable and wait to be ready for GPIO port F.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Set PF1, PF2 and PF3 as output.
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 );
}
