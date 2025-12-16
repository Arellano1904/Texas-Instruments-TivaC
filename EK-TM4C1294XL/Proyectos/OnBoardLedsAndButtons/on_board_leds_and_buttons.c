//*****************************************************************************
// Project to use the on board leds and buttons
// It used the interrupts on SW1 and SW2 to attend changes on leds states. 
// *****************************************************************************

/* LIBRARIES */
/* Common use libraries */
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header files //
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions //
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"

// Function declatarions // 
void buttonsConfig(void);
void ledsConfig(void);

int main(void){
    uint32_t ui32SysClock;

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_240), 120000000);

    // Invoque configuration functions // 
    ledsConfig();
    buttonsConfig();
    // Loop Forever
    while(1);
}

// Functions definitions //
void PortJ_Handler(void)
{
    uint32_t status;

    // Read interrupt status
    status = GPIOIntStatus(GPIO_PORTJ_BASE, true);

    // Clear interrupts immediately
    GPIOIntClear(GPIO_PORTJ_BASE, status);

    if(status & GPIO_PIN_0)
    {
        // SW1 pressed
        // Turn LEDs off 
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0x00);
    }

    if(status & GPIO_PIN_1)
    {
        // SW2 pressed
        // Turn LEDs on 
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x03);
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0x11);
    }
}

void ledsConfig(void){
    // Enable and wait on PORTN and PORTF enabling //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Configure PN0,PN1,PF0 and PF4 as output //
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // Turn LEDs off initially
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x03);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0x11);
}

void buttonsConfig(void){
    // Enable and wait on PORTJ enabling //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    // Configure PJ0 and PJ1 as input //
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Enable internal pull-ups //
    GPIOPadConfigSet(GPIO_PORTJ_BASE,
                     GPIO_PIN_0 | GPIO_PIN_1,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    // Disable interrupts during setup
    GPIOIntDisable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure interrupt type: falling edge
    GPIOIntTypeSet(GPIO_PORTJ_BASE,
                   GPIO_PIN_0 | GPIO_PIN_1,
                   GPIO_FALLING_EDGE);

    // Enable GPIO interrupt (NVIC)
    IntEnable(INT_GPIOJ);

    // Enable GPIO pin interrupts
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Global interrupt enable
    IntMasterEnable();
    
}

