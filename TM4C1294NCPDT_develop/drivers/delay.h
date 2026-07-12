#ifndef DELAY_H_
#define DELAY_H_
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"     // ASSERT() -> calls __error__() in DEBUG builds

//! Must be called once, immediately after SysCtlClockFreqSet(), before any
//! other module that delays. Pass it the value SysCtlClockFreqSet() returned.
extern void delay_init(uint32_t ui32SysClock);

//! Blocking delay. MINIMUM duration -- an ISR or the call overhead can only
//! ever make it longer. Max ~107,374 ms @ 120 MHz (clamped, never wraps).
extern void delay_ms(uint32_t ui32Ms);

//! Blocking delay. Max ~419,430 us @ 120 MHz (clamped). Fixed overhead is
//! ~20 cycles (~0.17 us @ 120 MHz), so short delays overshoot -- see notes.
extern void delay_us(uint32_t ui32Us);

#endif /* DELAY_H_ */

