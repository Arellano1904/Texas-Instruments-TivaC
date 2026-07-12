//*****************************************************************************
// Delay functions based on SysCtlDelay() and ui32SysClock 
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
#include "delay.h"

//*****************************************************************************
// SysCtlDelay() is hand-written assembly:
//
//      SysCtlDelay:
//          subs r0, #1          ; 1 cycle
//          bne  SysCtlDelay     ; 2 cycles when taken
//          bx   lr
//
// -> exactly 3 cycles per iteration, regardless of toolchain.
//
// The clock is captured once and turned into scale factors, so the hot path is
// a multiply and a shift -- NOT a 64-bit divide on every call. That matters:
// 1 us is only 120 cycles, and a runtime 64-bit divide can eat half of it.
//*****************************************************************************

static uint32_t g_ui32ItersPerMs   = 0;   // SysClock / 3000          -> 40,000 @ 120 MHz
static uint32_t g_ui32ItersPerUsQ8 = 0;   // (SysClock * 256) / 3e6   -> 10,240 @ 120 MHz
static uint32_t g_ui32MaxMs        = 0;   // largest ms that fits a uint32 iteration count
static uint32_t g_ui32MaxUs        = 0;   // ditto for us

//*****************************************************************************
// Why Q8 fixed point for the us factor, but plain integer for the ms factor?
//
// Truncation. At 16 MHz:
//      ms: 16e6/3000    = 5333.33 -> 5333   -> 0.006 % error   (harmless)
//      us: 16e6/3e6     =    5.33 ->    5   -> 6.25  % error   (NOT harmless)
//
// Scaling by 256 first keeps 8 fractional bits, dropping that 6.25 % to 0.06 %.
// At 120 MHz both divide exactly, so this costs nothing on your board -- but it
// means the module survives a clock change without silently going 6 % fast.
//*****************************************************************************
void delay_init(uint32_t ui32SysClock){
    ASSERT(ui32SysClock != 0U);

    g_ui32ItersPerMs   = ui32SysClock / 3000U;
    g_ui32ItersPerUsQ8 = (uint32_t)(((uint64_t)ui32SysClock * 256ULL) / 3000000ULL);

    // Precompute the ceilings so the hot path never needs a UDIV.
    g_ui32MaxMs = (g_ui32ItersPerMs   != 0U) ? (0xFFFFFFFFU / g_ui32ItersPerMs)   : 0U;
    g_ui32MaxUs = (g_ui32ItersPerUsQ8 != 0U) ? (0xFFFFFFFFU / g_ui32ItersPerUsQ8) : 0U;
}

//*****************************************************************************
void delay_ms(uint32_t ui32Ms){
    uint32_t ui32Iters;

    ASSERT(g_ui32ItersPerMs != 0U);   // delay_init() was never called
    ASSERT(ui32Ms <= g_ui32MaxMs);    // caller asked for more than ~107 s

    if(ui32Ms > g_ui32MaxMs){
        ui32Ms = g_ui32MaxMs;         // release-build clamp: saturate, never wrap
    }

    ui32Iters = g_ui32ItersPerMs * ui32Ms;

    // *** THE ONE THAT WILL HANG YOU ***
    // SysCtlDelay() decrements BEFORE it tests, so 0 wraps to 0xFFFFFFFF and the
    // loop runs 2^32 times -> ~107 SECONDS of dead CPU. Never let a zero through.
    if(ui32Iters == 0U){
        return;
    }

    SysCtlDelay(ui32Iters);
}

//*****************************************************************************
void delay_us(uint32_t ui32Us){
    uint32_t ui32Iters;

    ASSERT(g_ui32ItersPerUsQ8 != 0U);
    ASSERT(ui32Us <= g_ui32MaxUs);

    if(ui32Us > g_ui32MaxUs){
        ui32Us = g_ui32MaxUs;
    }

    // Hot path: one LDR, one MULS, one LSRS. ~6 cycles.
    ui32Iters = (g_ui32ItersPerUsQ8 * ui32Us) >> 8;

    if(ui32Iters == 0U){
        return;   // sub-iteration request (very low clock + tiny us). Same trap.
    }

    SysCtlDelay(ui32Iters);
}

