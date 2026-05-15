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
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
// Own driver libraries.
#include "drivers/ili9341_tm4c1294.h"


//*****************************************************************************
// Definitions.
//*****************************************************************************
/* ── PWM parameters ────────────────────────────────────────────────────────── */
#define PWM_FREQUENCY_HZ    1000UL          /* 20 kHz carrier               */
#define PWM_CLK_DIV         8UL              /* Must match PWMClockSet() arg  */
#define SYS_CLOCK_HZ        120000000UL
#define PWM_PERIOD_COUNTS   ((SYS_CLOCK_HZ / PWM_CLK_DIV) / PWM_FREQUENCY_HZ)
                                             /* = 750 counts                  */

/* ── ADC parameters ────────────────────────────────────────────────────────── */
#define ADC_SAMPLE_RATE_HZ  10000UL          /* 10 kHz sample rate            */
#define ADC_RESOLUTION      4095UL           /* 12-bit full-scale             */
#define ADC_HW_OVERSAMPLE   64               /* Hardware averaging (64×)      */

//*****************************************************************************
// Functions declarations.
//*****************************************************************************
/* ── Forward declaration (definition must live in startup_ccs.c vector table) */
void ADC1Seq3_Handler(void);
void PWM_Init(void);
void ADC_Init(void);
void Timer_Init(void);

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
/* ── Shared state (ISR → main) ─────────────────────────────────────────────── */
volatile uint32_t g_ui32ADCResult = 0;      /* Latest raw ADC reading        */
volatile bool     g_bNewSample    = false;  /* Set by ISR, cleared by main   */


//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void){
    // Run from the PLL at 120 MHz.
    systemClkFreq = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |SYSCTL_OSC_MAIN |SYSCTL_USE_PLL | SYSCTL_CFG_VCO_240), 120000000);
    // Display init.
    ili9341_init();
    ili9341_fill_screen(BLACK);
    // Init functions.
    PWM_Init();
    ADC_Init();
    Timer_Init();
    // Loop Forever
    while(1){
        if (g_bNewSample)
        {
            g_bNewSample = false;

            /*
             * Linear map:  ADC [0 … 4095]  →  pulse width [1 … PWM_PERIOD_COUNTS]
             *
             *   pw = 1  → ~0.13 % duty  (avoid pw = 0, which causes 100 % DC
             *                             in count-down generators)
             *   pw = PWM_PERIOD_COUNTS - 1  → ~99.87 % duty
             *
             * Modify this section to add dead-band, soft-start, or a
             * non-linear (e.g. look-up table) mapping as needed.
             */
            uint32_t ui32PW = ((uint32_t)g_ui32ADCResult *
                               (PWM_PERIOD_COUNTS - 1UL)) / ADC_RESOLUTION;

            if (ui32PW < 1UL)
            {
                ui32PW = 1UL;
            }

            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, ui32PW);
        }
    }
}

//*****************************************************************************
// Functions definitions.
//*****************************************************************************
/* ═══════════════════════════════════════════════════════════════════════════
 *  PWM_Init
 *  PK4 → M0PWM6  (PWM0, Generator 3, Output A)
 * ═══════════════════════════════════════════════════════════════════════════ */
void PWM_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)) {}

    /* PWM clock = SysClock / 8 = 15 MHz */
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);

    /* Mux PK4 to M0PWM6 */
    GPIOPinConfigure(GPIO_PK4_M0PWM6);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4);

    /* Count-down mode, no sync */
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
                    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    /* Period: 15 MHz / 20 kHz = 750 counts */
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWM_PERIOD_COUNTS);

    /* Start at 0 % — pulse width must be ≥ 1 */
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 1);

    PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, true);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ADC_Init
 *  ADC1, Sequencer 3 (single step), input AIN19 (PK3), Timer trigger
 * ═══════════════════════════════════════════════════════════════════════════ */
void ADC_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1)) {}
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)) {}

    /* PK3 → analog (disables digital function automatically) */
    GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);

    /*
     * Hardware averaging: 64 samples averaged per result.
     * Effective output rate = ADC_SAMPLE_RATE_HZ (timer triggers still 10 kHz
     * but each conversion averages 64 hardware samples internally).
     */
    ADCHardwareOversampleConfigure(ADC1_BASE, ADC_HW_OVERSAMPLE);

    /*
     * Sequencer 3 — single-step, lowest priority,
     * triggered by Timer0A.
     */
    ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_TIMER, 0);

    /* Step 0: AIN19, generate interrupt, mark end of sequence */
    ADCSequenceStepConfigure(ADC1_BASE, 3, 0,
                             ADC_CTL_CH19 | ADC_CTL_IE | ADC_CTL_END);

    ADCSequenceEnable(ADC1_BASE, 3);
    ADCIntClear(ADC1_BASE, 3);
    ADCIntEnable(ADC1_BASE, 3);

    /* Enable ADC1-SS3 in the NVIC (INT_ADC1SS3 from inc/hw_ints.h) */
    IntEnable(INT_ADC1SS3);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Timer_Init
 *  Timer0A in periodic mode — output compare triggers ADC1-SS3
 * ═══════════════════════════════════════════════════════════════════════════ */
void Timer_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) {}

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    /* Reload = SysClock / sample_rate = 120 000 000 / 10 000 = 12 000 */
    TimerLoadSet(TIMER0_BASE, TIMER_A,
                 (SYS_CLOCK_HZ / ADC_SAMPLE_RATE_HZ) - 1UL);

    /* Enable Timer0A → ADC trigger line */
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    TimerEnable(TIMER0_BASE, TIMER_A);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  ADC1Seq3_Handler  — ADC1 Sequencer-3 interrupt
 *
 *  This name MUST match the entry in the startup_ccs.c vector table.
 *  See startup_ccs.c → position INT_ADC1SS3 (vector 67).
 * ═══════════════════════════════════════════════════════════════════════════ */
void ADC1Seq3_Handler(void){
    /* Clear the sequencer interrupt flag */
    ADCIntClear(ADC1_BASE, 3);

    /* Read the single result from the FIFO */
    ADCSequenceDataGet(ADC1_BASE, 3, (uint32_t *)&g_ui32ADCResult);

    /* Signal main loop that fresh data is available */
    g_bNewSample = true;
}
