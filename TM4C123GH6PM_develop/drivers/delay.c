//*****************************************************************************
// Delay functions for TM4C123GH6PM -- hardware-timer timed, cycle-accurate
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
#include "delay.h"

//*****************************************************************************
// Why not SysCtlDelay()?
//
// SysCtlDelay() is a calibrated software loop -- "3 cycles per iteration" --
// and that figure only holds while flash feeds the pipeline one instruction
// per cycle. Above 40 MHz the TM4C123 flash needs a wait state, and a taken
// branch every two instructions is the prefetch buffer's worst case, so at
// 80 MHz each iteration really costs 4-5 cycles: every delay runs 30-65 %
// long even though the clock is a perfect 80 MHz.
//
// A GPTM in periodic mode counts actual system-clock cycles in hardware
// instead. It auto-reloads on timeout and the raw flag (TATORIS) latches
// until explicitly cleared, so consecutive periods butt together with no
// accumulated drift -- the software only observes, it never times anything
// itself.
//*****************************************************************************

static uint32_t g_ui32TicksPerMs = 0;   // SysClock / 1000  -> 80,000 @ 80 MHz
static uint32_t g_ui32TicksPerUs = 0;   // SysClock / 1e6   ->     80 @ 80 MHz

//*****************************************************************************
void delay_init(uint32_t ui32SysClock){
    ASSERT(ui32SysClock >= 1000000U);   // below 1 MHz, 1 us has no whole tick

    g_ui32TicksPerMs = ui32SysClock / 1000U;
    g_ui32TicksPerUs = ui32SysClock / 1000000U;

    // Enable and wait to be ready for the delay timer block.
    MAP_SysCtlPeripheralEnable(DELAY_TIMER_PERIPH);
    while(!MAP_SysCtlPeripheralReady(DELAY_TIMER_PERIPH));

    // Full-width (32-bit) periodic down-counter, clocked by the system clock.
    // Left disabled here; each delay call enables it for just its duration.
    MAP_TimerConfigure(DELAY_TIMER_BASE, TIMER_CFG_PERIODIC);
}

//*****************************************************************************
// Run the timer for ui32Count back-to-back periods of ui32Ticks cycles each.
// The RAW timeout flag is polled -- no NVIC, no ISR. HWREG is used in the hot
// loop instead of MAP_TimerIntStatus()/MAP_TimerIntClear() so the poll stays
// a few cycles, comfortably inside even a 1 us period (80 cycles @ 80 MHz).
// If an ISR holds us past a reload the latched flag stretches the delay --
// it can never shorten it.
//*****************************************************************************
static void timer_run(uint32_t ui32Ticks, uint32_t ui32Count){
    // Down-counter: loads ILR, hits 0 after ILR+1 cycles -> ILR = ticks - 1.
    MAP_TimerLoadSet(DELAY_TIMER_BASE, TIMER_A, ui32Ticks - 1U);
    HWREG(DELAY_TIMER_BASE + TIMER_O_ICR) = TIMER_ICR_TATOCINT;   // drop stale flag
    MAP_TimerEnable(DELAY_TIMER_BASE, TIMER_A);

    while(ui32Count--){
        while(!(HWREG(DELAY_TIMER_BASE + TIMER_O_RIS) & TIMER_RIS_TATORIS)){
        }
        HWREG(DELAY_TIMER_BASE + TIMER_O_ICR) = TIMER_ICR_TATOCINT;
    }

    MAP_TimerDisable(DELAY_TIMER_BASE, TIMER_A);   // idle between delays
}

//*****************************************************************************
void delay_ms(uint32_t ui32Ms){
    ASSERT(g_ui32TicksPerMs != 0U);   // delay_init() was never called

    if(ui32Ms == 0U){
        return;
    }
    timer_run(g_ui32TicksPerMs, ui32Ms);
}

//*****************************************************************************
void delay_us(uint32_t ui32Us){
    ASSERT(g_ui32TicksPerUs != 0U);

    if(ui32Us == 0U){
        return;
    }
    timer_run(g_ui32TicksPerUs, ui32Us);
}
