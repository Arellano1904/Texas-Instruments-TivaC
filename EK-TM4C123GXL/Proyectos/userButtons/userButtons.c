//*****************************************************************************
//
// Project to use te leds and buttons user onboard.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"

// Declaracion de funciones
void configButtons(void);
void configLeds(void);
void configInts();
void portfHandler(void);

//*****************************************************************************
//
//! \addtogroup example_list
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//
//*****************************************************************************
uint8_t counter = 0;

int
main(void)
{   
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    // Enable and wait for the port F to be ready for access
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    configButtons();
    configLeds();
    configInts();
    
    // Loop Forever
    while(1)
    {
        
    }
}

// Function definitions:
void configButtons(void){
    // ===== UNLOCK PF0 (SW2) =====
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;   // Unlock key
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= GPIO_PIN_0;   // Commit
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    // Set PF4 and PF0 as input (SW1 and SW2) 
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0);
    // 3. Enable internal pull-up resistor (for active-low buttons)
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    // 1. Set the interrupt type (e.g., falling edge). GPIO_FALLING_EDGE is standard for an active-low button press
    GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0, GPIO_FALLING_EDGE);
    // 2. Clear any pending interrupt flags for the pins. This is critical to prevent an immediate, false trigger
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0);
    // 3. Unmask the interrupt for the specific pins.This enables the pin to generate an interrupt
    GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4 | GPIO_PIN_0);


}

void configLeds(void){
    // Set PF1 as output (LED red)
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 );
    
}
void configInts(void){
    // 1. Register your custom function as the Interrupt Service Routine (ISR) for Port F
    // The name "portfHandler" is the standard symbol used in the startup file.
    GPIOIntRegister(GPIO_PORTF_BASE, portfHandler);
    // 2. Enable the GPIO Port F interrupt in the NVIC
    IntEnable(INT_GPIOF);
    // 3. Enable interrupts globally
    IntMasterEnable();
}

void portfHandler(){
    // Read and clear the interrupt status
    uint32_t ui32Status = GPIOIntStatus(GPIO_PORTF_BASE, true);
    
    // Check if the interrupt came from PF4 (SW1)
    if (ui32Status & GPIO_PIN_4)
    {
        // Turn on the GREEN LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x08 );
        // Clears the interrupt triger
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
    }
    if (ui32Status & GPIO_PIN_0)
    {
        // Turn on the RED LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0x04 );
        // Clears the interrupt triger
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    }
}
