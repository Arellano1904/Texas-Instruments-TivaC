//*****************************************************************************
// xpt2046 driver for 2.4_tft_display, ILI9341_driver, tm4c1294_spi2_uDMA
//*****************************************************************************
// xpt2046_tm4c Touch Driver
#include "xpt2046_tm4c1294.h"
// The inc folder contains the device header files for each TM4C device as well as the hardware header.
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
//*****************************************************************************
// //***** Definitions *****//
//*****************************************************************************
// XPT2046 control bytes: start=1, channel select, MODE=0 (12-bit),
// SER/DFR=0 (differential), PD1:PD0=00 (power down between conversions so
// PENIRQ stays enabled).
#define XPT2046_CMD_X       0x90    // X position (A2:A0 = 001)
#define XPT2046_CMD_Y       0xD0    // Y position (A2:A0 = 101)
// Samples averaged per axis to suppress jitter.
#define XPT2046_SAMPLES     8
// Raw 12-bit ADC values seen at the panel edges. Calibrate these per unit.
#define XPT2046_X_MIN       20
#define XPT2046_X_MAX       4076
#define XPT2046_Y_MIN       20
#define XPT2046_Y_MAX       4076
// Active area of the 2.4" panel in the current (portrait) orientation.
#define XPT2046_SCREEN_W    240
#define XPT2046_SCREEN_H    320

extern volatile uint8_t touch_asserted;

// Last touch position, in screen pixels (portrait-flipped 240x320 panel).
volatile uint16_t touch_x = 0;
volatile uint16_t touch_y = 0;

//*****************************************************************************
// //***** Functions definitions *****//
//*****************************************************************************

void xpt2046_enable(void){
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x00);
}

void xpt2046_disable(void){
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0);
}
// Clock one channel: a 16-bit command frame followed by a 16-bit frame that
// shifts out the BUSY bit + the 12 data bits. The command sits in the low byte
// so the leading zero byte acts as the start-bit gap; on the next frame the
// result lands left-aligned, so (rx >> 3) drops BUSY and the 3 trailing bits.
// Returns the averaged 0..4095 sample. CS is owned by the caller so several
// channels can share a single assertion.
static uint16_t xpt2046_read_raw(uint8_t cmd){
    uint32_t rx;
    uint32_t acc = 0;
    uint8_t i;
    for(i = 0; i < XPT2046_SAMPLES; i++){
        MAP_SSIDataPut(SSI2_BASE, (uint16_t)cmd);   // send command
        MAP_SSIDataGet(SSI2_BASE, &rx);             // discard command echo
        MAP_SSIDataPut(SSI2_BASE, 0x0000);          // clock out the result
        MAP_SSIDataGet(SSI2_BASE, &rx);
        acc += (rx >> 3) & 0x0FFF;
    }
    return (uint16_t)(acc / XPT2046_SAMPLES);
}

// Linear map of a raw reading onto [0, out_max], clamped at the edges.
static uint16_t xpt2046_scale(uint16_t raw, uint16_t in_min, uint16_t in_max, uint16_t out_max){
    if(raw <= in_min){ return 0; }
    if(raw >= in_max){ return out_max; }
    return (uint16_t)(((uint32_t)(raw - in_min) * out_max) / (in_max - in_min));
}

void xpt2046_request_coordinates(void){
    uint32_t flush;
    uint16_t raw_x, raw_y;

    // Reading the controller makes PENIRQ pulse, which would otherwise queue a
    // spurious touch interrupt and keep touch_asserted stuck set. Mask PL2 for
    // the duration of the transfer.
    MAP_GPIOIntDisable(GPIO_PORTL_BASE, GPIO_PIN_2);

    xpt2046_enable();                                       // CS low
    while(MAP_SSIDataGetNonBlocking(SSI2_BASE, &flush)){}   // drop stale RX data

    raw_x = xpt2046_read_raw(XPT2046_CMD_X);
    raw_y = xpt2046_read_raw(XPT2046_CMD_Y);

    while(MAP_SSIBusy(SSI2_BASE)){}                         // let the last frame finish
    xpt2046_disable();                                      // CS high

    // Discard the glitch the read produced, then re-arm the touch interrupt.
    MAP_GPIOIntClear(GPIO_PORTL_BASE, GPIO_PIN_2);
    MAP_GPIOIntEnable(GPIO_PORTL_BASE, GPIO_PIN_2);

    // Portrait-flipped panel: both axes run opposite to the raw ADC sweep, so
    // invert each mapped result. Adjust the *_MIN/*_MAX limits during
    // calibration; swap the channel->axis assignment if the panel is mounted
    // rotated 90 degrees.
    touch_x = (XPT2046_SCREEN_W - 1) - xpt2046_scale(raw_x, XPT2046_X_MIN, XPT2046_X_MAX, XPT2046_SCREEN_W - 1);
    touch_y = (XPT2046_SCREEN_H - 1) - xpt2046_scale(raw_y, XPT2046_Y_MIN, XPT2046_Y_MAX, XPT2046_SCREEN_H - 1);
}

void xpt2046_init(void){
    spi2_config();
}

void xpt2046_int_handler(void){
    // Read interrupt status from port L.
    uint32_t int_status = MAP_GPIOIntStatus(GPIO_PORTL_BASE, true);
    // Clear interrupts immediately
    MAP_GPIOIntClear(GPIO_PORTL_BASE, int_status);
    // Set touch flag
    if(int_status & GPIO_PIN_2){// Touch pressed
        touch_asserted = 0x01;
        return;
    }
}

void spi2_config(){
    // Enabling and wait for QSSI module and ports D and L to be ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL));
    // Configure pins for QSSI2
    // {PD3:Clk,PL0:CS,PD1:MOSI,PD0:MISO,PL1:rst,PL2:int}
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_1 | GPIO_PIN_3);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_0 | GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTL_BASE,GPIO_PIN_2);
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_0);
    MAP_GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    MAP_GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
    MAP_GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
    MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3 );
    // Enable internal pull-ups for touch interrupt //
    MAP_GPIOPadConfigSet(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    // Disable interrupts during setup
    MAP_GPIOIntDisable(GPIO_PORTL_BASE, GPIO_PIN_2);
    MAP_GPIOIntClear(GPIO_PORTL_BASE, GPIO_PIN_2);
    // Configure interrupt type: falling edge
    MAP_GPIOIntTypeSet(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_FALLING_EDGE);
    // Enable GPIO interrupt (NVIC)
    MAP_IntEnable(INT_GPIOL);
    // Enable GPIO pin interrupts
    MAP_GPIOIntEnable(GPIO_PORTL_BASE, GPIO_PIN_2);
    // Config QSSI2 module, master mode, 16bits len.
    MAP_SSIDisable(SSI2_BASE);
    SSIClockSourceSet(SSI2_BASE, SSI_CLOCK_SYSTEM);
    MAP_SSIConfigSetExpClk(SSI2_BASE, systemClkFreq, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,2000000, 16);
    MAP_SSIEnable(SSI2_BASE);
}

