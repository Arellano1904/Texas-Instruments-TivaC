//*****************************************************************************
// C file to use on board buttons and RGB led on EK-TM4C123GXL
// You must add int_buttons_handler as extern into the startup file to buttons work
//*****************************************************************************
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Own drivers
#include "on_board_buttons_and_led.h"

//*****************************************************************************
// VARS
//*****************************************************************************
static volatile uint8_t buttons_status = 0x00;

//*****************************************************************************
// FUNCTIONS DEFINITIONS
//*****************************************************************************
void config_buttons(void){
    // Enable and wait to be ready for GPIO port F.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // ===== UNLOCK PF0 (SW2) =====
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;   // Unlock key
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= GPIO_PIN_0;   // Commit
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    // Configure PF0 and PF4 as inputs.
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // 1. Enable internal pull-up resistor (for active-low buttons)
    MAP_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
    // 2. Set the interrupt type (e.g., falling edge). GPIO_FALLING_EDGE is standard for an active-low button press
    MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_FALLING_EDGE);
    // 3. Clear any pending interrupt flags for the pins. This is critical to prevent an immediate, false trigger
    MAP_GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // 4. Unmask the interrupt for the specific pins.This enables the pin to generate an interrupt
    MAP_GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // 5. Enable the GPIO Port F interrupt in the NVIC
    MAP_IntEnable(INT_GPIOF);
        // Enable the processor to service interrupts.
    MAP_IntMasterEnable();
}

void int_buttons_handler(void){
    // Read and clear the interrupt status
    uint32_t int_status = MAP_GPIOIntStatus(GPIO_PORTF_BASE, true);
    MAP_GPIOIntClear(GPIO_PORTF_BASE, int_status);
    if (int_status & SW2){ // Switch 2
        buttons_status = SW2;
        return;
    }
    if (int_status & SW1){ // Switch 1
        buttons_status = SW1;
        return;
    }
}

uint8_t pressed_button(void){
    uint8_t button_pressed = buttons_status;
    buttons_status = 0x00;
    return button_pressed;
}

void config_rgb_led(void){
    // Enable and wait to be ready for GPIO port F.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Set PF1, PF2 and PF3 as output.
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 );
}
