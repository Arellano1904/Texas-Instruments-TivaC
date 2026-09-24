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
#define ST7735_SWRESET  0x01
#define ST7735_SLPOUT   0x11
#define ST7735_NORON    0x13
#define ST7735_INVOFF   0x20
#define ST7735_DISPON   0x29
#define ST7735_CASET    0x2A
#define ST7735_RASET    0x2B
#define ST7735_RAMWR    0x2C
#define ST7735_MADCTL   0x36
#define ST7735_COLMOD   0x3A
#define ST7735_FRMCTR1  0xB1
#define ST7735_FRMCTR2  0xB2
#define ST7735_FRMCTR3  0xB3
#define ST7735_INVCTR   0xB4
#define ST7735_PWCTR1   0xC0
#define ST7735_PWCTR2   0xC1
#define ST7735_PWCTR3   0xC2
#define ST7735_PWCTR4   0xC3
#define ST7735_PWCTR5   0xC4
#define ST7735_VMCTR1   0xC5
#define ST7735_GMCTRP1  0xE0
#define ST7735_GMCTRN1  0xE1

#define SSI3_FRQ 20000000u
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
void st7735_snd_dt(uint8_t data);
void st7735_snd_cmd(uint8_t cmd);

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
    gpio_cnfg();


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
    // PORTD and PORTE for SSI3 peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    // Confgiure PD0 as SSI3Clk and PD3 as SSI3TX
    MAP_GPIOPinConfigure(GPIO_PD0_SSI3CLK);
    MAP_GPIOPinConfigure(GPIO_PD3_SSI3TX);
    MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_3);
    // Configure PE1, PD1 and PD2 as output for RST, CS and DC on SSI3 display
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_PIN_1 | GPIO_PIN_2);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1);
}
void spi_dsp_cnfg(void){
    // Enable and wait for SSI peripheral ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3));
    // Configure SSI2 module: Master mode, 16bit len, 15MHZ
    MAP_SSIDisable(SSI3_BASE);
    SSIClockSourceSet(SSI3_BASE,SSI_CLOCK_SYSTEM);
    MAP_SSIConfigSetExpClk(SSI3_BASE,SysClkFrq, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,SSI3_FRQ, 8);
    MAP_SSIEnable(SSI3_BASE);
}

void spi_dsp_len(uint8_t len){ // Change the SSI3 data len
    MAP_SSIDisable(SSI3_BASE);
    MAP_SSIConfigSetExpClk(SSI3_BASE,SysClkFrq, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,SSI3_FRQ, len);
    MAP_SSIEnable(SSI3_BASE);
}

// Display functions
void st7735_init(void){
    // Init needed functions and hardware reset
    delay_init(SysClkFrq);
    spi_dsp_cnfg();
    st7735_rst();
    // Commands and data needed to config the display
    // Wake up
    st7735_snd_cmd(ST7735_SWRESET);           
    delay_ms(150);
    st7735_snd_cmd(ST7735_SLPOUT);            
    delay_ms(255);
    //Frame rate control
    st7735_snd_cmd(ST7735_FRMCTR1);//normal mode
    st7735_snd_dt(0x01); 
    st7735_snd_dt(0x2C); 
    st7735_snd_dt(0x2D);
    st7735_snd_cmd(ST7735_FRMCTR2);//idle mode
    st7735_snd_dt(0x01); 
    st7735_snd_dt(0x2C); 
    st7735_snd_dt(0x2D);
    st7735_snd_cmd(ST7735_FRMCTR3);//partial mode
    st7735_snd_dt(0x01); 
    st7735_snd_dt(0x2C); 
    st7735_snd_dt(0x2D);
    st7735_snd_dt(0x01); 
    st7735_snd_dt(0x2C); 
    st7735_snd_dt(0x2D);
    st7735_snd_cmd(ST7735_INVCTR);//display inversion control
    st7735_snd_dt(0x07);
    //Power control + VCOM  
    st7735_snd_cmd(ST7735_PWCTR1);
    st7735_snd_dt(0xA2); 
    st7735_snd_dt(0x02); 
    st7735_snd_dt(0x84);
    st7735_snd_cmd(ST7735_PWCTR2);
    st7735_snd_dt(0xC5);
    st7735_snd_cmd(ST7735_PWCTR3);
    st7735_snd_dt(0x0A); 
    st7735_snd_dt(0x00);
    st7735_snd_cmd(ST7735_PWCTR4);
    st7735_snd_dt(0x8A); 
    st7735_snd_dt(0x2A);
    st7735_snd_cmd(ST7735_PWCTR5);
    st7735_snd_dt(0x8A); 
    st7735_snd_dt(0xEE);
    st7735_snd_cmd(ST7735_VMCTR1);//VCOM voltage
    st7735_snd_dt(0x0E);
    st7735_snd_cmd(ST7735_INVOFF);//non-inverted colors
 
    /*  Orientation + pixel format 
     *  MADCTL bit 3 = RGB/BGR order: flip it if R and B look swapped.
     *  Bits MY/MX/MV rotate & mirror the display */
    st7735_snd_cmd(ST7735_MADCTL);
    st7735_snd_dt(0xC8);//common default for red-tab 1.8"
 
    st7735_snd_cmd(ST7735_COLMOD);
    st7735_snd_dt(0x05);//0x05 = 16 bit/px, RGB565
 
    /* --- 6. Address window: 1.8" red-tab = 128x160, zero offset --------- *
     *  If you see a garbage border, your panel is a different tab variant
     *  and needs a +2/+1 or +2/+3 offset added to these start values.      */
    st7735_snd_cmd(ST7735_CASET);// columns 0..127
    st7735_snd_dt(0x00); st7735_snd_dt(0x00);// start = 0
    st7735_snd_dt(0x00); st7735_snd_dt(0x7F);// end   = 127
    st7735_snd_cmd(ST7735_RASET); // rows 0..159 
    st7735_snd_dt(0x00); st7735_snd_dt(0x00);// start = 0
    st7735_snd_dt(0x00); st7735_snd_dt(0x9F);// end   = 159
 
    //Gamma correction (voltage -> brightness curve)
    st7735_snd_cmd(ST7735_GMCTRP1);// positive gamma
    st7735_snd_dt(0x02); st7735_snd_dt(0x1C); st7735_snd_dt(0x07); st7735_snd_dt(0x12);
    st7735_snd_dt(0x37); st7735_snd_dt(0x32); st7735_snd_dt(0x29); st7735_snd_dt(0x2D);
    st7735_snd_dt(0x29); st7735_snd_dt(0x25); st7735_snd_dt(0x2B); st7735_snd_dt(0x39);
    st7735_snd_dt(0x00); st7735_snd_dt(0x01); st7735_snd_dt(0x03); st7735_snd_dt(0x10);
    st7735_snd_cmd(ST7735_GMCTRN1);// negative gamma 
    st7735_snd_dt(0x03); st7735_snd_dt(0x1D); st7735_snd_dt(0x07); st7735_snd_dt(0x06);
    st7735_snd_dt(0x2E); st7735_snd_dt(0x2C); st7735_snd_dt(0x29); st7735_snd_dt(0x2D);
    st7735_snd_dt(0x2E); st7735_snd_dt(0x2E); st7735_snd_dt(0x37); st7735_snd_dt(0x3F);
    st7735_snd_dt(0x00); st7735_snd_dt(0x00); st7735_snd_dt(0x02); st7735_snd_dt(0x10);
 
    // Turn the panel on 
    st7735_snd_cmd(ST7735_NORON);             
    delay_ms(10);// normal display on
    st7735_snd_cmd(ST7735_DISPON);            
    delay_ms(100);// display on
    // Change SSI3 len to 16bits
    spi_dsp_len(16);
}
void st7735_enable(void){
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,0x00);
}
void st7735_disable(void){
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,GPIO_PIN_1);
}
void st7735_rst(void){
    MAP_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,0x00);
    delay_ms(20);
    MAP_GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_1,GPIO_PIN_1);
    delay_ms(120);
}
void st7735_snd_dt(uint8_t data){
    st7735_enable();
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,GPIO_PIN_2);
    MAP_SSIDataPut(SSI3_BASE,data);
    while(MAP_SSIBusy(SSI3_BASE));
    st7735_disable();
}
void st7735_snd_cmd(uint8_t cmd){
    st7735_enable();
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2,0x00);
    MAP_SSIDataPut(SSI3_BASE,cmd);
    while(MAP_SSIBusy(SSI3_BASE));
    st7735_disable();
}
