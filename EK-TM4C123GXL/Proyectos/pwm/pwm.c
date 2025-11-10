//*****************************************************************************
//
// Project to show how to use the PWM module on TivaC 123G
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"

// Declaracion de funciones
void configPWM(void);
void configADC(void);
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
int
main(void)
{   
    // Reloj a 80 MHz (PLL con cristal de 16MHz)
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    // Invocamos funciones de configuracion
    configPWM();
    configADC();

    // Vatiables relevantes al PWM.
    uint32_t pwmClock = SysCtlClockGet() / 64; // Clock PWM
    uint32_t pwmFreq = 20000;  // 10 kHz
    uint32_t load = (pwmClock / pwmFreq) - 1;
    uint32_t adcValue;
    uint32_t duty;
    
    // Loop Forever
    while(1)
    {
        // Leer ADC
        ADCProcessorTrigger(ADC0_BASE, 3);
        while(!ADCIntStatus(ADC0_BASE, 3, false));
        ADCIntClear(ADC0_BASE, 3);
        ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);

        // adcValue: 0 a 4095 → duty en %
        duty = (adcValue * 100) / 4095;

        // Ajustar PWM: PWMPulseWidthSet = (load * porcentaje) / 100
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (load * duty) / 100);
    }
}

// Definicion de funciones.
void configPWM(void){
    // Habilitar PWM0 y GPIOB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // PB4 como M0PWM2
    GPIOPinConfigure(GPIO_PB4_M0PWM2);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);

    // Divisor del reloj del PWM
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // Configurar generador 1 (PWM0 Gen1 controla PWM2 y PWM3)
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ((SysCtlClockGet() / 64 ) / 20000));

    // Habilitar el generador y la salida
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
}

void configADC(void){
    // -------------------- CONFIG ADC (PE3 - AIN0) ------------
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

}
