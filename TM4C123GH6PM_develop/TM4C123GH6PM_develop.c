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
#include "inc/hw_ssi.h"
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions. 
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/udma.h"
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
// ST7735 commands
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
// RGB565 basic color format
#define BLACK       0x0000
#define NAVY        0x000F
#define DARKGREEN   0x03E0
#define DARKCYAN    0x03EF
#define MAROON      0x7800
#define PURPLE      0x780F
#define OLIVE       0x7BE0
#define LIGHTGREY   0xC618
#define DARKGREY    0x7BEF
#define BLUE        0x001F
#define GREEN       0x07E0
#define CYAN        0x07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define ORANGE      0xFD20
#define GREENYELLOW 0xAFE5
#define PINK        0xFC18 
// Display features
// Portrait orientation (panel native 128x160; MADCTL MV bit left clear).
#define ST7735_WIDTH   128
#define ST7735_HEIGHT  160
// SSI3 freq
#define SSI3_FRQ 20000000u
//*****************************************************************************
// GLOBALS VARIABLES
//*****************************************************************************
// Tabla completa ASCII 5×7 (Adafruit font) //
const uint8_t font5x7[] = {
  0x00,0x00,0x00,0x00,0x00, // 32 space
  0x00,0x00,0x5F,0x00,0x00, // 33 !
  0x00,0x07,0x00,0x07,0x00, // 34 "
  0x14,0x7F,0x14,0x7F,0x14, // 35 #
  0x24,0x2A,0x7F,0x2A,0x12, // 36 $
  0x23,0x13,0x08,0x64,0x62, // 37 %
  0x36,0x49,0x55,0x22,0x50, // 38 &
  0x00,0x05,0x03,0x00,0x00, // 39 '
  0x00,0x1C,0x22,0x41,0x00, // 40 (
  0x00,0x41,0x22,0x1C,0x00, // 41 )
  0x14,0x08,0x3E,0x08,0x14, // 42 *
  0x08,0x08,0x3E,0x08,0x08, // 43 +
  0x00,0x50,0x30,0x00,0x00, // 44 ,
  0x08,0x08,0x08,0x08,0x08, // 45 -
  0x00,0x60,0x60,0x00,0x00, // 46 .
  0x20,0x10,0x08,0x04,0x02, // 47 /
  0x3E,0x51,0x49,0x45,0x3E, // 48 0
  0x00,0x42,0x7F,0x40,0x00, // 49 1
  0x42,0x61,0x51,0x49,0x46, // 50 2
  0x21,0x41,0x45,0x4B,0x31, // 51 3
  0x18,0x14,0x12,0x7F,0x10, // 52 4
  0x27,0x45,0x45,0x45,0x39, // 53 5
  0x3C,0x4A,0x49,0x49,0x30, // 54 6
  0x01,0x71,0x09,0x05,0x03, // 55 7
  0x36,0x49,0x49,0x49,0x36, // 56 8
  0x06,0x49,0x49,0x29,0x1E, // 57 9
  0x00,0x36,0x36,0x00,0x00, // 58 :
  0x00,0x56,0x36,0x00,0x00, // 59 ;
  0x08,0x14,0x22,0x41,0x00, // 60 <
  0x14,0x14,0x14,0x14,0x14, // 61 =
  0x00,0x41,0x22,0x14,0x08, // 62 >
  0x02,0x01,0x51,0x09,0x06, // 63 ?
  0x32,0x49,0x79,0x41,0x3E, // 64 @
  0x7E,0x11,0x11,0x11,0x7E, // 65 A
  0x7F,0x49,0x49,0x49,0x36, // 66 B
  0x3E,0x41,0x41,0x41,0x22, // 67 C
  0x7F,0x41,0x41,0x22,0x1C, // 68 D
  0x7F,0x49,0x49,0x49,0x41, // 69 E
  0x7F,0x09,0x09,0x09,0x01, // 70 F
  0x3E,0x41,0x49,0x49,0x7A, // 71 G
  0x7F,0x08,0x08,0x08,0x7F, // 72 H
  0x00,0x41,0x7F,0x41,0x00, // 73 I
  0x20,0x40,0x41,0x3F,0x01, // 74 J
  0x7F,0x08,0x14,0x22,0x41, // 75 K
  0x7F,0x40,0x40,0x40,0x40, // 76 L
  0x7F,0x02,0x04,0x02,0x7F, // 77 M
  0x7F,0x04,0x08,0x10,0x7F, // 78 N
  0x3E,0x41,0x41,0x41,0x3E, // 79 O
  0x7F,0x09,0x09,0x09,0x06, // 80 P
  0x3E,0x41,0x51,0x21,0x5E, // 81 Q
  0x7F,0x09,0x19,0x29,0x46, // 82 R
  0x46,0x49,0x49,0x49,0x31, // 83 S
  0x01,0x01,0x7F,0x01,0x01, // 84 T
  0x3F,0x40,0x40,0x40,0x3F, // 85 U
  0x1F,0x20,0x40,0x20,0x1F, // 86 V
  0x7F,0x20,0x18,0x20,0x7F, // 87 W
  0x63,0x14,0x08,0x14,0x63, // 88 X
  0x07,0x08,0x70,0x08,0x07, // 89 Y
  0x61,0x51,0x49,0x45,0x43, // 90 Z
  0x00,0x7F,0x41,0x41,0x00, // 91 [
  0x02,0x04,0x08,0x10,0x20, // 92 backslash
  0x00,0x41,0x41,0x7F,0x00, // 93 ]
  0x04,0x02,0x01,0x02,0x04, // 94 ^
  0x80,0x80,0x80,0x80,0x80, // 95 _
  0x00,0x03,0x07,0x00,0x00, // 96 `
  0x20,0x54,0x54,0x54,0x78, // 97 a
  0x7F,0x48,0x44,0x44,0x38, // 98 b
  0x38,0x44,0x44,0x44,0x20, // 99 c
  0x38,0x44,0x44,0x48,0x7F, // 100 d
  0x38,0x54,0x54,0x54,0x18, // 101 e
  0x08,0x7E,0x09,0x01,0x02, // 102 f
  0x0C,0x52,0x52,0x52,0x3E, // 103 g
  0x7F,0x08,0x04,0x04,0x78, // 104 h
  0x00,0x44,0x7D,0x40,0x00, // 105 i
  0x20,0x40,0x44,0x3D,0x00, // 106 j
  0x7F,0x10,0x28,0x44,0x00, // 107 k
  0x00,0x41,0x7F,0x40,0x00, // 108 l
  0x7C,0x04,0x18,0x04,0x78, // 109 m
  0x7C,0x08,0x04,0x04,0x78, // 110 n
  0x38,0x44,0x44,0x44,0x38, // 111 o
  0x7C,0x14,0x14,0x14,0x08, // 112 p
  0x08,0x14,0x14,0x18,0x7C, // 113 q
  0x7C,0x08,0x04,0x04,0x08, // 114 r
  0x48,0x54,0x54,0x54,0x20, // 115 s
  0x04,0x3F,0x44,0x40,0x20, // 116 t
  0x3C,0x40,0x40,0x20,0x7C, // 117 u
  0x1C,0x20,0x40,0x20,0x1C, // 118 v
  0x3C,0x40,0x30,0x40,0x3C, // 119 w
  0x44,0x28,0x10,0x28,0x44, // 120 x
  0x0C,0x50,0x50,0x50,0x3C, // 121 y
  0x44,0x64,0x54,0x4C,0x44, // 122 z
  0x00,0x08,0x36,0x41,0x00, // 123 {
  0x00,0x00,0x7F,0x00,0x00, // 124 |
  0x00,0x41,0x36,0x08,0x00, // 125 }
  0x10,0x08,0x08,0x10,0x08  // 126 ~
};
uint32_t SysClkFrq = 0x00000000;
// For uDMA use
#pragma DATA_ALIGN(uDMAControlTable, 1024);
uint8_t uDMAControlTable[1024];
// Vector to store the comands and data to initialize the ST7735 display


//*****************************************************************************
// FUNCTION DECLARATIONS
//*****************************************************************************
// Peripheral config functions
void gpio_cnfg(void);
void dma_dsp_cnfg(void);
void dma_dsp_snd_buffer(uint16_t* buffer, uint32_t bufferLen);
void spi_dsp_cnfg(void);
void spi_dsp_len(uint8_t len);
// Display functions
void st7735_init(void);
void st7735_enable(void);
void st7735_disable(void);
void st7735_rst(void);
void st7735_snd_dt(uint8_t data);
void st7735_snd_cmd(uint8_t cmd);
void st7735_st_wndw();
void st7735_fll_scrn();
void st7735_drw_chr();
void st7735_prtn_str();
void st7735_prtn_int();
void st7735_prtn_float();

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

void dma_dsp_cnfg(void){
    // Enable uDMA peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA));
    // Enable uDMA controller
    MAP_uDMAEnable();
    // Set base address of control table
    MAP_uDMAControlBaseSet(uDMAControlTable);
    // Enable uDMA on SSI3 TX
    MAP_SSIDMAEnable(SSI3_BASE, SSI_DMA_TX);
    // Optional but safe
    MAP_uDMAChannelAssign(UDMA_CH15_SSI3TX);
    // Disable channel before configuration
    MAP_uDMAChannelDisable(UDMA_CH15_SSI3TX);
    // Configure channel
    MAP_uDMAChannelControlSet(UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
        UDMA_SIZE_16 |        // 16-bit data (one RGB565 pixel per item)
        UDMA_SRC_INC_16 |     // walk through the source buffer
        UDMA_DST_INC_NONE |   // SSI3 data register stays put
        UDMA_ARB_8);          // re-arbitrate every 8 items (SSI FIFO depth)
    // Transfers are polled to completion in display_snd_dma_buffer(), so the
    // uDMA completion interrupt is left disabled on purpose (no ISR exists).

}

void dma_dps_snd_buffer(uint16_t* buffer, uint32_t bufferLen){
    // CS low to enable the Display and DC high to data mode //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1 | GPIO_PIN_2, 0x04);
    // Disable channel before setup
    MAP_uDMAChannelDisable(UDMA_CH15_SSI3TX);
    // Configure transfer
    MAP_uDMAChannelTransferSet(
       UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
        buffer,
        (void *)(SSI3_BASE + SSI_O_DR),
        bufferLen
    );
    // Start transfer
    MAP_uDMAChannelEnable(UDMA_CH15_SSI3TX);
    // Wait for completion
    while(MAP_uDMAChannelIsEnabled(UDMA_CH15_SSI3TX));
    // Wait for SSI to finish shifting last word
    while(MAP_SSIBusy(SSI3_BASE));
    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
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
