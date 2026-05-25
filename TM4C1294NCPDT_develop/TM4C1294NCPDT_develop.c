//*****************************************************************************
// Boost converter.
//*****************************************************************************

//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common used libraries.
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header.
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h" 
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/adc.h"
// Own driver libraries.
#include "drivers/ili9341_tm4c1294.h"

//*****************************************************************************
// Definitions.
//*****************************************************************************
#define SYSCLK_HZ 120000000
#define ADC_RESOLUTION 4096.0f 
#define ADC_REf 3.3f
#define TIMER_LOAD (SYSCLK_HZ/ 1000) // 1ms
//*****************************************************************************
// Functions declarations.
//*****************************************************************************
void adc0ssq3_config(void);
void adc0ssq3_handler(void);
void timer0_config(void);
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
// Global variables.
//*****************************************************************************
volatile uint32_t systemClkFreq;
uint32_t adc0Ssq3Value = 0x0000;
volatile float adcValue = 0.0f;
volatile uint8_t adc_ready = 0x00;

//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void){
    // Run from the PLL at 120 MHz.
    systemClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);
    // Floating point unit enable
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // Global interrupt enable
    MAP_IntMasterEnable();  
    // Display init //
    ili9341_init();
    ili9341_fill_screen(BLACK);
    ili9341_print_string(0, 0, "TivaC: EK-TM4C1294XL", RED, BLACK);
    ili9341_print_string(0, 16, "2.4SpiDisplay : ILI9341", RED, BLACK);
    ili9341_print_string(0, 32, "Voltaje: ", BLUE,BLACK);
    
    // ADC config function
    adc0ssq3_config();
    // Timer
    timer0_config();
               
    // Loop Forever
    while(1){
        if(adc_ready){
        adc_ready == 0x00;
        adcValue = ((float)adc0Ssq3Value / ADC_RESOLUTION) * ADC_REf;
        ili9341_print_float(65, 32, adcValue, 2, BLUE, BLACK);
        }
    }
}

//*****************************************************************************
// Functions definitions.
//*****************************************************************************

void timer0_config(void){
    // Enable peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
    // Configure the timer
    MAP_TimerDisable(TIMER0_BASE, TIMER_A);
    MAP_TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
    MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, (TIMER_LOAD - 1));
    // Enable triger event for ADC
    MAP_TimerControlTrigger(TIMER0_BASE, TIMER_A, true);
    MAP_TimerEnable(TIMER0_BASE, TIMER_A);
}

void adc0ssq3_config(void){
    // Enable the ADC module and related peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    // Wait for the ADC0 module to be ready
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    // Disable digital function on the pin
    MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    // Disable sequencer 3 during config
    MAP_ADCSequenceDisable(ADC0_BASE, 3);
    // Enable the first sample sequencer to capture the value of channel 0 when the processor trigger occurs
    MAP_ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);
    // Usa VDDA (3.3 V)
    MAP_ADCReferenceSet(ADC0_BASE, ADC_REF_INT);
    // Enable ADC0 Sequencer 3 interrupt    
    MAP_ADCIntEnable(ADC0_BASE, 3);
    // Clear ADC0 Sequencer 3 interrupt
    MAP_ADCIntClear(ADC0_BASE, 3);
    // Enable interrupt in NVIC
    MAP_IntEnable(INT_ADC0SS3);
    MAP_ADCSequenceStepConfigure(ADC0_BASE, 3, 0,ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0);
    MAP_ADCSequenceEnable(ADC0_BASE, 3);

}
// ADC0 Sequencer 3 interruptio handler
void adc0ssq3_handler(void){
    // ALWAYS clear first, before reading
    MAP_ADCIntClear(ADC0_BASE, 3);     
    adc_ready = 0x01;
    MAP_ADCSequenceDataGet(ADC0_BASE, 3, &adc0Ssq3Value);
}


