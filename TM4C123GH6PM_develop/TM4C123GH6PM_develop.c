//*****************************************************************************
// Main develop file for TM4C123GH6PM
//*****************************************************************************
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
// Common use libraries
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header.

// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions. 
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
// Own drivers
#include "drivers/on_board_buttons_and_led.h"
#include "drivers/delay.h"

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line){
}
#endif
//*****************************************************************************
// DEFINES
//*****************************************************************************
#define SSI2_FRQ 10000u
//*****************************************************************************
// GLOBALS VARIABLES
//*****************************************************************************
uint32_t SysClkFrq = 0x00000000;
// Vector to store the comands and data to initialize the ST7735 display


//*****************************************************************************
// FUNCTION DECLARATIONS
//*****************************************************************************
// Peripheral config functions
void gpio_cnfg(void);
void dma_dsp_cnfg(void);
void spi_dsp_cnfg(void);
void spi_dsp_len(uint8_t len);
// Display functions
void st7735_init(void);
void st7735_enable(void);
void st7735_disable(void);
void st7735_rst(void);
void st7735_data(void);
void st7735_cmd(void);

//*****************************************************************************
// Main 'C' Language entry point.  Toggle the RGB LED with the on board buttons.
//*****************************************************************************
int main(void){
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    SysClkFrq = SysCtlClockGet();
    // Floating point unit enabling //
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // Configure the on board buttons (interrupt driven) and the RGB LED.
    config_buttons();
    config_rgb_led();
    // Display


    // Loop Forever
    while(1){
        // Read the latched event ONCE: button_pressed() clears it on read,
        // so a second call would see 0 and lose the event.
        uint8_t buttons = pressed_button();
        if(buttons & SW1){      // SW1 pressed.
            MAP_GPIOPinWrite(GPIO_PORTF_BASE,LEDR,LEDR);
            st7735_init();
        }
        if(buttons & SW2){      // SW2 pressed.
            
            MAP_GPIOPinWrite(GPIO_PORTF_BASE,LEDR,0x00);
        }
    }
}

//*****************************************************************************
// FUNCTION DEFINITIONS
//*****************************************************************************
void gpio_cnfg(void){ // Enable and wait for GPIO used on this project
    // PORTB for SSI2 peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    // Confgiure PB4 as SSI2Clk and PB7 as SSi2TX
    MAP_GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    MAP_GPIOPinConfigure(GPIO_PB7_SSI2TX);
    // Configure PB0, PB5 and PB6 as output for RST, DC and CS on SSI2 display
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_0 | GPIO_PIN_5 | GPIO_PIN_6);
}
void spi_dsp_cnfg(void){
    // Enable and wait for SSI peripheral ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI2));
    // Configure SSI2 module: Master mode, 16bit len, 15MHZ
    MAP_SSIDisable(SSI2_BASE);
    SSIClockSourceSet(SSI2_BASE,SSI_CLOCK_SYSTEM);
    MAP_SSIConfigSetExpClk(SSI2_BASE,SysClkFrq, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,SSI2_FRQ, 8);
    MAP_SSIEnable(SSI2_BASE);
}

void spi_dsp_len(uint8_t len){ // Change the SSI2 data len
    MAP_SSIDisable(SSI2_BASE);
    MAP_SSIConfigSetExpClk(SSI2_BASE,SysClkFrq, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,SSI2_FRQ, len);
    MAP_SSIEnable(SSI2_BASE);
}

// Display functions
void st7735_init(void){
    delay_init(SysClkFrq);
    spi_dsp_cnfg();
    st7735_rst();
    
    spi_dsp_len(16);
}
void st7735_enable(void){
    MAP_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6,0x00);
}
void st7735_disable(void){
    MAP_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_6,GPIO_PIN_6);
}
void st7735_rst(void){
    MAP_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,0x00);
    delay_ms(20);
    MAP_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_PIN_0);
    delay_ms(120);
}
void st7735_snd_data(uint8_t data){
    st7735_enable();
    MAP_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_PIN_5);
    MAP_SSIDataPut(SSI2_BASE,data);
    while(MAP_SSIBusy(SSI2_BASE));
    display_disable();
}
void st7735_snd_cmd(uint8_t cmd){
    st7735_enable();
    MAP_GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_5,0x00);
    MAP_SSIDataPut(SSI2_BASE,cmd);
    while(MAP_SSIBusy(SSI2_BASE));
    display_disable();
}
