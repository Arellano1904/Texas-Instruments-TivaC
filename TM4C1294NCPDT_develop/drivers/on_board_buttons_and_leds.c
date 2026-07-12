//*****************************************************************************
// You must add int_butons_handler as extern fuction on startup_ccs.c
// file and link it as port J interrupt handler
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
#include "on_board_buttons_and_leds.h"

//*****************************************************************************
// VARS
//*****************************************************************************
static volatile uint8_t buttons_status = 0x00;

//*****************************************************************************
// FUNCTIONS DEFINITIONS
//*****************************************************************************
// Buttons
void config_buttons(void){
    // Enable and wait for buttons port to be ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    // Configure PJ0 and PJ1 as input //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, SW1 | SW2);
    // Enable internal pull-ups //
    MAP_GPIOPadConfigSet(GPIO_PORTJ_BASE,SW1 | SW2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    // Disable interrupts during setup
    MAP_GPIOIntDisable(GPIO_PORTJ_BASE, SW1 | SW2);
    MAP_GPIOIntClear(GPIO_PORTJ_BASE, SW1 | SW2);
    // Configure interrupt type: falling edge
    MAP_GPIOIntTypeSet(GPIO_PORTJ_BASE,SW1 | SW2,GPIO_FALLING_EDGE);
    // Enable GPIO interrupt (NVIC)
    MAP_IntEnable(INT_GPIOJ);
    // Enable GPIO pin interrupts
    MAP_GPIOIntEnable(GPIO_PORTJ_BASE, SW1 | SW2);
    // Global ints enabling //
    MAP_IntMasterEnable();
}

uint8_t pressed_button(void){
    uint8_t button_pressed = buttons_status;
    buttons_status = 0x00;
    return button_pressed;
}

void int_buttons_handler(void){
    // Read interrupt status from port J.
    uint32_t int_status = MAP_GPIOIntStatus(GPIO_PORTJ_BASE, true);
    // Clear interrupts immediately
    MAP_GPIOIntClear(GPIO_PORTJ_BASE, int_status);
    if(int_status & SW1){// SW1 pressed
        buttons_status = SW1; 
        return;
    }

    if(int_status & SW2){// SW2 pressed
        buttons_status = SW2;
        return;
    }
}

// Leds
void config_leds(void){
    // Enable and wait for PORTN and PORTF to be ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Configure PN0,PN1,PF0 and PF4 as output
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, LED0 | LED1);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED2 | LED3);
    // Turn LEDs on initially
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, LED0 | LED1, 0x00);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, LED2 | LED3, 0x00);
}

