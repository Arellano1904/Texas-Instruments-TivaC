//*****************************************************************************
// Test project. 
// *****************************************************************************

// Colores básicos en formato RGB565
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


/* LIBRARIES */
/* Common use libraries */
#include <stdint.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header files //
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions //
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"

// Tabla completa ASCII 5×7 (Adafruit font) //
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

// Function declatarions //
// Timer functions // 
void Timer0A_Handler(void); 
void timer0Config(void);
void delay_ms(uint32_t ms);
// Buttons functions //
void PortJ_Handler(void);
void buttonsConfig(void);
// Leds functions // 
void ledsConfig(void);
// SPI functions //
void spi0Config(uint32_t clkFreq);
// Display functions // 
void initDisplay(uint32_t clkFreq);
void resetDisplay(uint32_t clkFreq);
void sendDisplayCommand(uint8_t cmd);
void sendDisplayData(uint8_t data);
void setDisplayWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void drawSinglePixel(uint16_t x, uint16_t y, uint16_t color);
void drawChar(uint16_t x, uint16_t y, char c,uint16_t color, uint16_t bg);
void drawString(uint16_t x, uint16_t y,const char *str,uint16_t color, uint16_t bg);
void fillScreen(uint16_t color);
void intToStr(int32_t value, char *buf);
void drawInt(uint16_t x, uint16_t y,int32_t value,uint16_t color, uint16_t bg);
void drawFloat(uint16_t x, uint16_t y,float value, uint8_t decimals, uint16_t color, uint16_t bg);


int main(void){
    
    uint32_t ui32SysClock;

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_240), 120000000);

    // Invoque configuration functions // 
    ledsConfig();
    buttonsConfig();
    spi0Config(ui32SysClock);
    initDisplay(ui32SysClock);
    timer0Config();
    IntMasterEnable();

    
    fillScreen(RED);
    drawString(0, 0, "Hello TivaC TM4C1294NCPDT", BLACK, RED);
    // Loop Forever
    while(1){
    }
}
// Functions definitions //
void Timer0A_Handler(void){
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);  
}

void timer0Config(void){
    // Enable Timer0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    // Configure Timer0 as 32-bit periodic timer
    TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
    
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);

    //Start the timer
    //TimerEnable(TIMER0_BASE, TIMER_A);
}

void delay_ms(uint32_t ms)
{
    // 120 MHz → 120000 cycles per ms
    uint32_t load = (ms * 120000) - 1;

    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerLoadSet(TIMER0_BASE, TIMER_A, load);

    TimerEnable(TIMER0_BASE, TIMER_A);

    // WAIT here
    while (!TimerIntStatus(TIMER0_BASE, false));

}

void PortJ_Handler(void){
    uint32_t status;

    // Read interrupt status
    status = GPIOIntStatus(GPIO_PORTJ_BASE, true);

    // Clear interrupts immediately
    GPIOIntClear(GPIO_PORTJ_BASE, status);

    if(status & GPIO_PIN_0)
    {
        // SW1 pressed
    drawInt(0, 10, 12345, WHITE, BLACK);
    drawInt(0, 20, -6789, GREEN, BLACK);

    }

    if(status & GPIO_PIN_1)
    {
        // SW2 pressed
        drawFloat(0, 30, 3.1416f, 3, CYAN, BLACK);
        drawFloat(0, 40, -12.75f, 2, YELLOW, BLACK);

    }
}

void ledsConfig(void){
    // Enable and wait on PORTN and PORTF enabling //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    // Configure PN0,PN1,PF0 and PF4 as output //
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // Turn LEDs off initially
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1, 0x00);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0x00);
}

void buttonsConfig(void){
    // Enable and wait on PORTJ enabling //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ));
    // Configure PJ0 and PJ1 as input //
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Enable internal pull-ups //
    GPIOPadConfigSet(GPIO_PORTJ_BASE,
                     GPIO_PIN_0 | GPIO_PIN_1,
                     GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPU);
    // Disable interrupts during setup
    GPIOIntDisable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure interrupt type: falling edge
    GPIOIntTypeSet(GPIO_PORTJ_BASE,
                   GPIO_PIN_0 | GPIO_PIN_1,
                   GPIO_FALLING_EDGE);

    // Enable GPIO interrupt (NVIC)
    IntEnable(INT_GPIOJ);

    // Enable GPIO pin interrupts
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
}

void spi0Config(uint32_t clkFreq){
    // Emabling and wait fot periphs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    // Configure PA0, PA4 and PA5 for QSSI0 (CLK,MOSI,MISO)
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
    // Configure GPIO Port A to use some pins as QSSI mode
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5);
    // Configure PA3, PA6 and PA7 as CS, DC and RST
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);
    // Config QSSI0
    SSIConfigSetExpClk(SSI0_BASE, clkFreq, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,20000000, 8);
    // Enable QSSI0
    SSIEnable(SSI0_BASE);
}

void initDisplay(uint32_t clkFreq){
    resetDisplay(clkFreq);
    sendDisplayCommand(0x01); // Software reset
    SysCtlDelay(clkFreq/30);
    sendDisplayCommand(0x11); // Sleep out
    SysCtlDelay(clkFreq/30);
    sendDisplayCommand(0x3A); // Pixel Format
    sendDisplayData(0x55); // RGB565
    sendDisplayCommand(0x36); // MADCTL
    sendDisplayData(0xC0); // RGB, 180 degrees rotation
    sendDisplayCommand(0x29);
    SysCtlDelay(clkFreq/30);

}

void resetDisplay(uint32_t clkFreq){
    // RESET pin to low //
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, 0);
    //
    SysCtlDelay(clkFreq/30);
    // RESTE pin to high //
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_PIN_7);
    SysCtlDelay(clkFreq/30);
}

void sendDisplayCommand(uint8_t cmd){
    // CS and DC to low (Enable the Display and put it in command mode) //
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_6, 0);
    // Send the command to the SPI0 //
    SSIDataPut(SSI0_BASE, cmd);
    // Wait for data to be tansfered //
    while(SSIBusy(SSI0_BASE));
    // CS high to disabble the display //
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_6, GPIO_PIN_6);
}

void sendDisplayData(uint8_t data){
    // CS low to enable the Display and DC high to data mode //
    GPIOPinWrite(GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_6, 0x08);
    // Send the data to the SPI3 //
    SSIDataPut(SSI0_BASE, data);
    // Wait for data to be tansfered //
    while(SSIBusy(SSI0_BASE));
    // CS high to disabble the display //
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, GPIO_PIN_6);
}

void setDisplayWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
    sendDisplayCommand(0x2A);// Column addr set
    sendDisplayData(x0 >> 8);
    sendDisplayData(x0 & 0xFF);
    sendDisplayData(x1 >> 8);
    sendDisplayData(x1 & 0xFF);

    sendDisplayCommand(0x2B);// Page addr set
    sendDisplayData(y0 >> 8);
    sendDisplayData(y0 & 0xFF);
    sendDisplayData(y1 >> 8);
    sendDisplayData(y1 & 0xFF);

    sendDisplayCommand(0x2C);// Memory write
}

void drawSinglePixel(uint16_t x, uint16_t y, uint16_t color)
{
    setDisplayWindow(x, y, x, y);

    sendDisplayData(color >> 8);
    sendDisplayData(color & 0xFF);
}

void drawChar(uint16_t x, uint16_t y, char c,
                      uint16_t color, uint16_t bg)
{
    if (c < 32 || c > 126) return;
    uint8_t row, col; 
    for (col = 0; col < 5; col++)
    {
        uint8_t line = Font5x7[(c - 32) * 5 + col];

        for (row = 0; row < 8; row++)
        {
            if (line & 0x01)
                drawSinglePixel(x + col, y + row, color);
            else
                drawSinglePixel(x + col, y + row, bg);

            line >>= 1;
        }
    }
}

void drawString(uint16_t x, uint16_t y,
                        const char *str,
                        uint16_t color, uint16_t bg)
{
    while (*str)
    {
        drawChar(x, y, *str, color, bg);
        x += 6; // 5 pixels + 1 spacing
        str++;
    }
}


void fillScreen(uint16_t color){
    uint32_t pixels = ILI9341_WIDTH * ILI9341_HEIGHT;

    setDisplayWindow(0, 0,
                          ILI9341_WIDTH - 1,
                          ILI9341_HEIGHT - 1);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;

    while (pixels--)
    {
        sendDisplayData(hi);
        sendDisplayData(lo);
    }
}

void intToStr(int32_t value, char *buf)
{
    char tmp[12];
    int i = 0, j = 0;

    if (value == 0)
    {
        buf[0] = '0';
        buf[1] = '\0';
        return;
    }

    if (value < 0)
    {
        buf[j++] = '-';
        value = -value;
    }

    while (value > 0)
    {
        tmp[i++] = (value % 10) + '0';
        value /= 10;
    }

    while (i > 0)
        buf[j++] = tmp[--i];

    buf[j] = '\0';
}

void floatToStr(float value, char *buf, uint8_t decimals)
{
    int32_t intPart;
    float frac;
    int i = 0,j = 0;

    if (value < 0)
    {
        buf[i++] = '-';
        value = -value;
    }

    intPart = (int32_t)value;
    frac = value - intPart;

    // Integer part
    char intBuf[12];
    intToStr(intPart, intBuf);

    for (j = 0; intBuf[j]; j++)
        buf[i++] = intBuf[j];

    buf[i++] = '.';

    // Fractional part
    while (decimals--)
    {
        frac *= 10.0f;
        int digit = (int)frac;
        buf[i++] = digit + '0';
        frac -= digit;
    }

    buf[i] = '\0';
}


void drawInt(uint16_t x, uint16_t y,
                     int32_t value,
                     uint16_t color, uint16_t bg)
{
    char buf[12];
    intToStr(value, buf);
    drawString(x, y, buf, color, bg);
}

void drawFloat(uint16_t x, uint16_t y,
                       float value, uint8_t decimals,
                       uint16_t color, uint16_t bg)
{
    char buf[20];
    floatToStr(value, buf, decimals);
    drawString(x, y, buf, color, bg);
}
