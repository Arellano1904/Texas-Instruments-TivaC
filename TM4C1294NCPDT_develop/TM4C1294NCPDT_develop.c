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
#define ADC_SCALE (3.3f / 4095.0f)
#define TIMER_LOAD (SYSCLK_HZ/ 1000) // 1ms
#define PWM_FREQ 20000
#define PWM_CLK (SYSCLK_HZ / 2)  // PWM_SYSCLK_DIV_2 divides by 2
#define PWM_LOAD (PWM_CLK / PWM_FREQ)
//*****************************************************************************
// Functions declarations.
//*****************************************************************************
void adc0ssq3_config(void);
void adc0ssq3_handler(void);
void timer0_config(void);
void pwm0_config(void);
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
uint32_t pwmLoad = 0x0000;
volatile float fDutyCycle = 0.50f;   // 50%
uint32_t ui32Width = 0x0000;

//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void){
    // Run from the PLL at 120 MHz.
    systemClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), SYSCLK_HZ);
    // Floating point unit enable
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // Display init //
    ili9341_init();
    ili9341_fill_screen(BLACK);
    ili9341_print_string(0, 0, "TivaC: EK-TM4C1294XL", RED, BLACK);
    ili9341_print_string(0, 16, "2.4SpiDisplay:ILI9341-240x320", RED, BLACK);
    ili9341_print_string(0, 32, "Voltaje: ", BLUE,BLACK);
    
    // ADC config function
    adc0ssq3_config();
    // Timer
    timer0_config();
    // PWM 
    pwm0_config();
    // Global interrupt enable
    MAP_IntMasterEnable();        
    // Loop Forever
    while(1){
        if(adc_ready){
        adc_ready == 0x00;
        adcValue = ((float)adc0Ssq3Value * ADC_SCALE);
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

void pwm0_config(void){
    // Enabling and waiting for related peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Configure pin PM3 as PWM output
    MAP_GPIOPinConfigure(GPIO_PF2_M0PWM2);
    MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    // Config PWM module clock
    MAP_PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_2);
    // Config PWM generator
    MAP_PWMGenConfigure(PWM0_BASE,PWM_GEN_1,PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_STOP);
    // Set PWM period
    MAP_PWMGenPeriodSet(PWM0_BASE,PWM_GEN_1,PWM_LOAD);
    // Set PWM pulse width
    ui32Width = (uint32_t)(PWM_LOAD * fDutyCycle);
    MAP_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui32Width);
    // Enable PWM output
    MAP_PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    MAP_PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}


