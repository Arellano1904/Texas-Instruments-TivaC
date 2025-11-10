// Librerias //
// Standart c libraries
#include <stdint.h>
#include <stdbool.h>
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/timer.h"
// The inc folder contains the device header files for each TM4C device as well as the hardware header files.
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"

// Function declarations
void cnfgTimer(void);
void cnfgBttns(void);
void cnfgInts(void);
void cnfgUart(void);
void portf_HandlerInts(void);
void UARTSend(char *);

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// Global variables
volatile uint32_t g_counter = 0;

// Main function

int main(){
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Invoque configuration functions
    cnfgBttns();
    cnfgUart();
    cnfgTimer();
    cnfgInts();

    UARTSend("** Starting main process **\n");

    // Infinite loop.
    while(1){
        if(g_counter >= 1000){
            UARTSend("Counter finished\n");
            g_counter = 0;          
        }
    }
}

// Function definitions
void cnfgBttns(void){
    // Enable and wait for the port F to be ready for access
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
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

void cnfgUart(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}

void cnfgTimer(void){
    // Enable Timer0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    // Configure Timer0 as 32-bit periodic timer
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Load value for 1 ms at 80 MHz
    // 80,000 cycles = 1 ms
    TimerLoadSet(TIMER0_BASE, TIMER_A, 80000 - 1);
}

void cnfgInts(void){
    // Register your custom function as the Interrupt Service Routine (ISR) for Port F
    // The name "portfHandler" is the standard symbol used in the startup file.
    GPIOIntRegister(GPIO_PORTF_BASE, portf_HandlerInts);
    // Enable the GPIO Port F interrupt in the NVIC
    IntEnable(INT_GPIOF);
    // Enable interrupt for Timer0A
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Finish enable interrupts globally
    IntMasterEnable();
}

void portf_HandlerInts(void){
    // Read and clear the interrupt status
    uint32_t ui32Status = GPIOIntStatus(GPIO_PORTF_BASE, true);
    
    // Check if the interrupt came from PF4 (SW1)
    if (ui32Status & GPIO_PIN_4)
    {   
        // Clears the interrupt triger
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
        // Start Timer
        TimerEnable(TIMER0_BASE, TIMER_A);
    }
    if (ui32Status & GPIO_PIN_0)
    {   // Printing info to uart0
        UARTSend("SW2\n");
        // Clears the interrupt triger
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    }
}

void timer0AHandler(void)
{   // Clear interrupt flag
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Do something every 1 ms
    g_counter++;
    
}

void UARTSend(char *str)
{
    while(*str)
    {
        UARTCharPut(UART0_BASE, *str++); // send one character
    }
}
