//*****************************************************************************
// Test project.
// *****************************************************************************
//***** Colores b�sicos en formato RGB565 *****//
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
#define PINK        0xF81F
#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320

//***** LIBRARIES *****//
// Common use libraries //
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header files //
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions //
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/fpu.h"
#include "driverlib/udma.h"

//***** Tabla completa ASCII 5�7 (Adafruit font) *****//
const uint8_t Font5x7[] = {
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

//***** Function declatarions *****//

// BUTTONS //
void portJHandler(void);
void buttonsConfig(void);

// LEDS //
void ledsConfig(void);

// QSSI  //
void spi0Config(uint32_t clkFreq);
void spi0bitMode(uint32_t clkFreq, uint32_t len);

// Display functions //
void ILI9341_reset(uint32_t systemClkFreq);
void ILI9341_send_command(uint8_t cmd);
void ILI9341_send_data(uint8_t data);
void ILI9341_init(uint32_t systemClkFreq);
void ILI9341_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1,uint32_t systemClkFreq);
void ILI9341_fill_screen(uint16_t color, uint32_t clkFreq);
void ILI9341_drawChar(uint16_t x,uint16_t y,char c,uint16_t fg,uint16_t bg,uint32_t clkFreq);
void ILI9341_printString(uint16_t x,uint16_t y,const char *str,uint16_t fg,uint16_t bg,uint32_t clkFreq);

// uDMA //
void uDMAConfig(void);
#pragma DATA_ALIGN(uDMAControlTable, 1024);
uint8_t uDMAControlTable[1024];
void uDMA_send_buffer(uint16_t* dataBuffer, uint32_t count);

//***** Flags variables *****//
uint8_t sw1State, sw2State = 0x00;

// Buffer to proof uDMA send function //
uint16_t colorBuffer[ILI9341_WIDTH];

//***** Main function *****//
int main(void){

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    uint32_t ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_240), 120000000);


    //***** Invoque configuration functions *****//
    // Floating point unit enabling //
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    // LEDS //
    ledsConfig();
    // BUTTONS //
    buttonsConfig();
    // QSSI //
    spi0Config(ui32SysClock);
    // uDMA //
    uDMAConfig();
    // Global ints enabling //
    MAP_IntMasterEnable();
    // DISPLAY //
    ILI9341_init(ui32SysClock);
    ILI9341_fill_screen(BLACK, ui32SysClock);

    // Loop Forever
    while(1){
        if(sw1State){ // SW1 pressed
            sw1State = 0x0;
            ILI9341_drawChar(0,0,'A',RED,WHITE,ui32SysClock);
        }
        if(sw2State){// SW2 pressed
            sw2State = 0x00;
            ILI9341_printString(0,8,"Hello TivaC", RED,WHITE,ui32SysClock);
        }
    }
}

//***** Functions definitions *****//
// LEDS //
void ledsConfig(void){
    // Enable and wait on PORTN and PORTF enabling //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Configure PN0,PN1,PF0 and PF4 as output //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // Turn LEDs off initially
    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);
    MAP_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0x00);
}
// BUTTONS //
void portJHandler(void){
    uint32_t status;

    // Read interrupt status
    status = MAP_GPIOIntStatus(GPIO_PORTJ_BASE, true);

    // Clear interrupts immediately
    MAP_GPIOIntClear(GPIO_PORTJ_BASE, status);

    if(status & GPIO_PIN_0){// SW1 pressed
        // Turn on on board led 0
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x02);
        sw1State = 0x01;

    }

    if(status & GPIO_PIN_1){// SW2 pressed
        // Turn off on board led 0
        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x00);
        sw2State = 0x01;
    }
}
void buttonsConfig(void){
    // Enable and wait on PORTJ enabling //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    // Configure PJ0 and PJ1 as input //
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Enable internal pull-ups //
    MAP_GPIOPadConfigSet(GPIO_PORTJ_BASE,
                     GPIO_PIN_0 | GPIO_PIN_1,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    // Disable interrupts during setup
    MAP_GPIOIntDisable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    MAP_GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure interrupt type: falling edge
    MAP_GPIOIntTypeSet(GPIO_PORTJ_BASE,
                   GPIO_PIN_0 | GPIO_PIN_1,
                   GPIO_FALLING_EDGE);

    // Enable GPIO interrupt (NVIC)
    MAP_IntEnable(INT_GPIOJ);

    // Enable GPIO pin interrupts
    MAP_GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);

}
// QSSI //
void spi0Config(uint32_t clkFreq){
    // Emabling and wait fot periphs
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    // Configure PA2, PA4 and PA5 for QSSI0 (CLK,MOSI,MISO)
    MAP_GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    MAP_GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    MAP_GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);

    // Configure GPIO Port A to use some pins as QSSI mode
    MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
    // Configure PA3, PA6 and PA7 as DC, CS and RST
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);
    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
    // Config QSSI0 as 8bit len
    // Disable QSSI0
    MAP_SSIDisable(SSI0_BASE);
    // Configure the bits len
    MAP_SSIConfigSetExpClk(SSI0_BASE, clkFreq, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,30000000, 16);
    // Enable QSSI0
    MAP_SSIEnable(SSI0_BASE);
}

void spi0bitMode(uint32_t clkFreq, uint32_t len){
    // Disable QSSI0
    MAP_SSIDisable(SSI0_BASE);
    // Configure the bits len
    MAP_SSIConfigSetExpClk(SSI0_BASE, clkFreq, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,30000000, len);
    // Enable QSSI0
    MAP_SSIEnable(SSI0_BASE);
}

// DISPLAY //
void ILI9341_reset(uint32_t systemClkFreq){
    // GPION7 to low for resete //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
    MAP_SysCtlDelay(systemClkFreq/30);
    // RESTE pin to high //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    MAP_SysCtlDelay(systemClkFreq/30);
}

void ILI9341_send_command(uint8_t cmd){
    // CS and DC to low (Enable the Display and put it in command mode) //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_6, 0x00);
    // Send the command to the SPI0 //
    MAP_SSIDataPut(SSI0_BASE, cmd);
    // Wait for data to be tansfered //
    while(MAP_SSIBusy(SSI0_BASE));
    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6, GPIO_PIN_6);
}

void ILI9341_send_data(uint8_t data){
    // CS low to enable the Display and DC high to data mode //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_6, 0x08);
    // Send the data to the SPI3 //
    MAP_SSIDataPut(SSI0_BASE, data);
    // Wait for data to be tansfered //
    while(MAP_SSIBusy(SSI0_BASE));
    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
}

void ILI9341_init(uint32_t systemClkFreq){
    ILI9341_reset(systemClkFreq);
    // Change to 8bit len to send commands
    spi0bitMode(systemClkFreq, 8);
    ILI9341_send_command(0x01); // Software reset
    MAP_SysCtlDelay(systemClkFreq/30);
    ILI9341_send_command(0x11); // Sleep out
    MAP_SysCtlDelay(systemClkFreq/30);
    ILI9341_send_command(0x3A); // Pixel Format
    ILI9341_send_data(0x55); // RGB565
    ILI9341_send_command(0x36); // MADCTL
    ILI9341_send_data(0xC0); // RGB, 180 degrees rotation
    ILI9341_send_command(0x29);
    // Change to 16 bit len for send pixels data
    spi0bitMode(systemClkFreq, 16);
    MAP_SysCtlDelay(systemClkFreq/30);
}

void ILI9341_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1,uint32_t systemClkFreq){
    // Commands and address parameters MUST be sent in 8-bit mode
    spi0bitMode(systemClkFreq, 8);

    // Column Address Set (CASET)
    ILI9341_send_command(0x2A);
    ILI9341_send_data(x0 >> 8);     // X start high
    ILI9341_send_data(x0 & 0xFF);   // X start low
    ILI9341_send_data(x1 >> 8);     // X end high
    ILI9341_send_data(x1 & 0xFF);   // X end low

    // Page Address Set (PASET)
    ILI9341_send_command(0x2B);
    ILI9341_send_data(y0 >> 8);     // Y start high
    ILI9341_send_data(y0 & 0xFF);   // Y start low
    ILI9341_send_data(y1 >> 8);     // Y end high
    ILI9341_send_data(y1 & 0xFF);   // Y end low

    // Switch back to 16-bit mode for pixel streaming
    spi0bitMode(systemClkFreq, 16);

    // Write to RAM
    ILI9341_send_command(0x2C);
}

void ILI9341_fill_screen(uint16_t color, uint32_t clkFreq){
    uint32_t i;

    // Prepare one line
    for(i = 0; i < ILI9341_WIDTH; i++)
        colorBuffer[i] = color;

    // Set full window
    ILI9341_set_window(0, 0,
        ILI9341_WIDTH - 1,
        ILI9341_HEIGHT - 1,
        clkFreq);

    // Stream lines using uDMA
    for(i = 0; i < ILI9341_HEIGHT; i++)
    {
        uDMA_send_buffer(colorBuffer, ILI9341_WIDTH);
    }
}

void ILI9341_drawChar(uint16_t x,uint16_t y,char c,uint16_t fg,uint16_t bg,uint32_t clkFreq){
    uint16_t lineBuffer[6];
    uint8_t col, row;
    const uint8_t *glyph;

    if(c < 32 || c > 127)
        c = '?';

    glyph = &Font5x7[(c - 32) * 5];

    // Set window for ONE character
    ILI9341_set_window(x, y, x + 5, y + 7, clkFreq);

    for(row = 0; row < 7; row++)
    {
        for(col = 0; col < 5; col++)
        {
            if(glyph[col] & (1 << row))
                lineBuffer[col] = fg;
            else
                lineBuffer[col] = bg;
        }

        // spacing column
        lineBuffer[5] = bg;

        uDMA_send_buffer(lineBuffer, 6);
    }

    // Last spacing row
    for(col = 0; col < 6; col++)
        lineBuffer[col] = bg;

    uDMA_send_buffer(lineBuffer, 6);
}

void ILI9341_printString(uint16_t x,uint16_t y,const char *str,uint16_t fg,uint16_t bg,uint32_t clkFreq){
    while(*str)
    {
        ILI9341_drawChar(x, y, *str++, fg, bg, clkFreq);
        x += 6;

        // simple wrap
        if(x + 6 >= ILI9341_WIDTH)
        {
            x = 0;
            y += 8;
        }
    }
}

// uDMA //
void uDMAConfig(void){
    // Enable uDMA peripheral
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_UDMA));

    // Enable uDMA controller
    MAP_uDMAEnable();

    // Set base address of control table
    MAP_uDMAControlBaseSet(uDMAControlTable);

    // Enable uDMA on SSI0 TX
    MAP_SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);

    // Optional but safe
    MAP_uDMAChannelAssign(UDMA_CH11_SSI0TX);

    // Disable channel before configuration
    MAP_uDMAChannelDisable(UDMA_CH11_SSI0TX);

    // Use BURST mode
    //uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_USEBURST);

    // Configure channel
    MAP_uDMAChannelControlSet(UDMA_CH11_SSI0TX | UDMA_PRI_SELECT,
        UDMA_SIZE_16 |        // 16-bit data
        UDMA_SRC_INC_16 |     // increment source
        UDMA_DST_INC_NONE |   // SSI FIFO
        UDMA_ARB_8);          // burst of 16 words
}


void uDMA_send_buffer(uint16_t* dataBuffer, uint32_t count){
    // CS low, DC high (DATA mode)
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_6, 0x08);

    // Disable channel before setup
    MAP_uDMAChannelDisable(UDMA_CH11_SSI0TX);

    // Configure transfer
    MAP_uDMAChannelTransferSet(
        UDMA_CH11_SSI0TX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
        dataBuffer,
        (void *)(SSI0_BASE + SSI_O_DR),
        count
    );

    // Enable channel
    MAP_uDMAChannelEnable(UDMA_CH11_SSI0TX);

    // Wait for completion
    while(MAP_uDMAChannelIsEnabled(UDMA_CH11_SSI0TX));

    // Wait for SSI to finish shifting last word
    while(MAP_SSIBusy(SSI0_BASE));

    // CS high
    MAP_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
}

