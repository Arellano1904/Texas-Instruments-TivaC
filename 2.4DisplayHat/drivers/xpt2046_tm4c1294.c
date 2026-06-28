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

extern volatile uint8_t touch_asserted;

//*****************************************************************************
// //***** Functions definitions *****//
//*****************************************************************************

void xpt2046_enable(void){
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, 0x00);
}

void xpt2046_disable(void){
    MAP_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0);
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
    MAP_GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_2);
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

