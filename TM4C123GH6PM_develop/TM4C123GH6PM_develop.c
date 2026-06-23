//*****************************************************************************
// Develop file
//*****************************************************************************

//*****************************************************************************
// Libraries
//*****************************************************************************
// Common use libraries
#include <stdint.h>
#include <stdbool.h>
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions. 
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
// The inc folder contains the device header files for each TM4C device as well as the hardware header files.
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
// Own driver libraries
#include "drivers/1.8_st7735_display.h"

//*****************************************************************************
// Useful definitions.
//*****************************************************************************

//***** Flags variables *****//
uint8_t sw1State, sw2State = 0x00;

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
// Function declaration.
//*****************************************************************************

// On board buttons.
void buttons_config(void);
void buttons_handler(void);

// On board rgbLed
void rgb_led_config(void);


//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void)
{
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    
    // Floating point unit enabling //
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    
    // Invoque configuration functions
    buttons_config();
    rgb_led_config();
    spi3_config();
    uDMA_spi3_config();

    // Init display module
    st7735_init();
    // Enable interrupts globally
    MAP_IntMasterEnable();
    // Fille the screen
    st7735_fill_screen(BLACK);
    st7735_print_string(0, 0, "TivaC: TM4C123GH6PM", RED, BLACK);
    st7735_print_string(0, 10, "1.8SpiDisplay: ST7735", RED, BLACK);
    // Loop Forever
    while(1)
    {
        if(sw1State){ // SW1 pressed
            sw1State = 0x0;
            // Turn red led on.
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x02);
        }
        if(sw2State){// SW2 pressed
            sw2State = 0x00;
            // Turn blue led on.
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x04);
        }
    }
}

//*****************************************************************************
// Function definitions.
//*****************************************************************************
// On board buttons :
void buttons_config(void){
    // Enable and wait to be ready for GPIO port F.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // ===== UNLOCK PF0 (SW2) =====
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;   // Unlock key
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= GPIO_PIN_0;   // Commit
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    // Configure PF0 and PF4 as inputs.
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // 3. Enable internal pull-up resistor (for active-low buttons)
    MAP_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
    // 1. Set the interrupt type (e.g., falling edge). GPIO_FALLING_EDGE is standard for an active-low button press
    MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_FALLING_EDGE);
    // 2. Clear any pending interrupt flags for the pins. This is critical to prevent an immediate, false trigger
    MAP_GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // 3. Unmask the interrupt for the specific pins.This enables the pin to generate an interrupt
    MAP_GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // 4. Enable the GPIO Port F interrupt in the NVIC
    MAP_IntEnable(INT_GPIOF);
}
void buttons_handler(void){
    // Read and clear the interrupt status
    uint32_t ui32Status = MAP_GPIOIntStatus(GPIO_PORTF_BASE, true);
    
    if (ui32Status & GPIO_PIN_0) // Switch 2
    {    
        sw2State = 0x01;
        // Clears the interrupt triger
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    }
    if (ui32Status & GPIO_PIN_4) // Switch 1
    {
        sw1State = 0x01;
        // Clears the interrupt triger
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
    }
}

// On board rgbLed
void rgb_led_config(void){
    // Enable and wait to be ready for GPIO port F.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Set PF1, PF2 and PF3 as output.
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 ); 
}
