//*****************************************************************************
// //***** LIBRARIES *****//
//*****************************************************************************
// Common used libraries.
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header.
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
// ili9341DisplayDriver
#include "drivers/ili9341_tm4c1294.h"
#include "drivers/xpt2046_tm4c1294.h"

//*****************************************************************************
// //***** The error routine that is called if the driver library encounters an error *****//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
// //***** Function declaration *****//
//*****************************************************************************
// ON BOARD BUTTONS //
void buttons_config(void);
void buttons_ints_handler(void);
// ON BOARD LEDS //
void leds_config(void);

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line){}
#endif

//*****************************************************************************
// Global variables.
//*****************************************************************************
volatile uint32_t systemClkFreq = 0x00000000; // System clock freq.
volatile uint8_t sw1State, sw2State = 0x00;
volatile uint8_t touch_asserted = 0x00;

//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void){
    //***** Clock config to run at 120 MHz *****//
    systemClkFreq = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);
    //***** Invoque configuration functions *****//
    // Floating point unit enabling //
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // Buttons //
    buttons_config();
    // Leds //
    leds_config();
    // Global ints enabling //
    MAP_IntMasterEnable();
    // Init display //
    ili9341_init();
    xpt2046_init();
    ili9341_fill_screen(BLACK);
    ili9341_print_string(0, 0, "TivaC: EK-TM4C1294XL", RED, BLACK);
    ili9341_print_string(0, 16, "2.4SpiDisplay:ILI9341-240x320", RED, BLACK);
    // Static labels for the touch readout.
    ili9341_print_string(0, 40, "Touch X:", GREEN, BLACK);
    ili9341_print_string(0, 56, "Touch Y:", GREEN, BLACK);
    //***** Loop Forever *****//
    while(1){
        if(touch_asserted){ //Touch pressed
            touch_asserted = 0x00;
            MAP_GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0,GPIO_PIN_0);
            // Read the touch position and refresh the on-screen values.
            xpt2046_request_coordinates();
            ili9341_print_string(72, 40, "    ", BLACK, BLACK);   // clear stale digits
            ili9341_print_int(72, 40, touch_x, WHITE, BLACK);
            ili9341_print_string(72, 56, "    ", BLACK, BLACK);
            ili9341_print_int(72, 56, touch_y, WHITE, BLACK);
        }

    }
}

//*****************************************************************************
// Function definitions.
//*****************************************************************************
// BUTTONS //
void buttons_config(void){
    // Enable and wait on PORTJ enabling //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    // Configure PJ0 and PJ1 as input //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Enable internal pull-ups //
    MAP_GPIOPadConfigSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    // Disable interrupts during setup
    MAP_GPIOIntDisable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    MAP_GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Configure interrupt type: falling edge
    MAP_GPIOIntTypeSet(GPIO_PORTJ_BASE,GPIO_PIN_0 | GPIO_PIN_1,GPIO_FALLING_EDGE);
    // Enable GPIO interrupt (NVIC)
    MAP_IntEnable(INT_GPIOJ);
    // Enable GPIO pin interrupts
    MAP_GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
}

void buttons_ints_handler(void){
    // Read interrupt status from port J.
    uint32_t status = MAP_GPIOIntStatus(GPIO_PORTJ_BASE, true);
    // Clear interrupts immediately
    MAP_GPIOIntClear(GPIO_PORTJ_BASE, status);
    if(status & GPIO_PIN_0){// SW1 pressed
        sw1State = 0x01;
        return;
    }

    if(status & GPIO_PIN_1){// SW2 pressed
        sw2State = 0x01;
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 , GPIO_PIN_0);
        return;
    }
}
// LEDS //
void leds_config(void){
    // Enable and wait for PORTN and PORTF to be ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Configure PN0,PN1,PF0 and PF4 as output
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // Turn LEDs off initially
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0x00);
}

