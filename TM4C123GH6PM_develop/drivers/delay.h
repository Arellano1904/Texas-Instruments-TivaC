#ifndef DELAY_TM4C123_H_
#define DELAY_TM4C123_H_
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"        // HWREG()
#include "inc/hw_memmap.h"       // TIMERn_BASE
#include "inc/hw_timer.h"        // TIMER_O_RIS / TIMER_O_ICR raw-flag access
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"     // ASSERT() -> calls __error__() in DEBUG builds

//*****************************************************************************
// DEFINES
//*****************************************************************************
// The hardware timer this driver owns. Change BOTH lines to relocate it; any
// 16/32-bit (TIMERn) or 32/64-bit wide (WTIMERn) block works -- no pins used.
#define DELAY_TIMER_PERIPH  SYSCTL_PERIPH_TIMER5
#define DELAY_TIMER_BASE    TIMER5_BASE

//! Must be called once, immediately after SysCtlClockSet(), before any other
//! module that delays. Pass it SysCtlClockGet() (80,000,000 with SYSDIV_2_5 +
//! PLL). Claims DELAY_TIMER_PERIPH in full-width 32-bit periodic mode; its
//! interrupt is never enabled in the NVIC, the raw flag is polled instead.
extern void delay_init(uint32_t ui32SysClock);

//! Blocking delay, timed by the hardware timer counting real system-clock
//! cycles, so it is immune to flash wait states, compiler and optimization
//! level. MINIMUM duration -- an ISR can only ever stretch it. Full uint32
//! range accepted.
extern void delay_ms(uint32_t ui32Ms);

//! Blocking delay. Fixed setup overhead is a handful of cycles (well under
//! 1 us @ 80 MHz), so very short delays overshoot slightly, never undershoot.
extern void delay_us(uint32_t ui32Us);

#endif /* DELAY_TM4C123_H_ */
