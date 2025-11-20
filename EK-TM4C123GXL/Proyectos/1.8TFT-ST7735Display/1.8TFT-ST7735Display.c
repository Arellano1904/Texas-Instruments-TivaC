//*****************************************************************************
// Project to use a 1.8 TFT SPI Display (ST7735 driver)
// This is part of revision 2.2.0.295 of the EK-TM4C123GXL Firmware Package.
//*****************************************************************************

/* Pin Connections  
Display ----- EK-TM4C123GXL
  Vcc         3.3v
  GND         GND
  CS          PD1
  RESET       PB5
  A0(DC)      PD2
  SDA         PD3
  SCK         PD0
*/ 

// Colores básicos en formato RGB565
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define TFT_XSTART 0
#define TFT_YSTART 0

/* Common use libraries */
#include <stdint.h>
#include <stdbool.h>
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
// The inc folder contains the device header files for each TM4C device as well as the hardware header files.
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
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

/* Functions declaration  */
void configSPI3(void);
void initDisplay(void);
void resetDisplay(void);
void sendDisplayCommand(uint8_t);
void sendDisplayData(uint8_t);
void TFT_FillScreen(uint16_t);
void TFT_DrawPixel(uint16_t, uint16_t, uint16_t);
void TFT_DrawChar(int, int, char, uint16_t, uint16_t, uint8_t);
void TFT_DrawText(int, int, const char *, uint16_t, uint16_t, uint8_t);
void TFT_DrawNumber(int, int, int, uint16_t, uint16_t, uint8_t);
void TFT_SetAddressWindow(uint16_t, uint16_t, uint16_t, uint16_t);

//*****************************************************************************
// The error routine that is called if the driver library encounters an error.
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void){
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ| SYSCTL_OSC_MAIN);

    // Summon config functions 
    configSPI3();
    initDisplay();

    TFT_FillScreen(BLACK);
    TFT_DrawText(0, 0,"Hello TivaC", RED, BLACK, 1);
    TFT_DrawText(0, 10,"1.8-TFT-7735R", RED, BLACK, 1);
    // pruebas para diagnosticar
    TFT_DrawChar(0, 20, 'A', CYAN, BLACK, 1);   // letra 'A' (debe verse)
    TFT_DrawChar(0, 30, '1', CYAN, BLACK, 1);  // dígito '1' (debe verse)
    TFT_DrawNumber(0, 40, 1904, CYAN, BLACK, 1);   // número usando DrawNumber

    // Loop Forever
    while(1)
    {
       
    }
}

/* Function definition  */

void configSPI3(void){ // Configure hardware to use SPI3 //
    // Enable SSI3, PORT_D and PORT_B //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // Configure GPIO Port D pins 0 and 3 to be used as SSI
    GPIOPinTypeSSI(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_3);

    // Configure PD0, PD3 for SSI3 (CLK, TX) //
    GPIOPinConfigure(GPIO_PD0_SSI3CLK); 
    GPIOPinConfigure(GPIO_PD3_SSI3TX);

    // Configure ChipSelect (CS-PD1) and DataComand (DC-PD2) pins  //
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2);

    // Configure RESET pin (PB5) //
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,  GPIO_PIN_5);

    // Configure the SSI3 as SPI MODE0, 8-bit, 16 MHz // 
    SSIConfigSetExpClk(SSI3_BASE,SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,16000000,8);

    SSIEnable(SSI3_BASE);
    
}

void initDisplay(void){
    // No coments needed //
    resetDisplay();
    // Software reset //
    sendDisplayCommand(0x01); 
    // 10ms delay to wait for the display response //
    SysCtlDelay(SysCtlClockGet()/30);
    // Sleep out //
    sendDisplayCommand(0x11);
    SysCtlDelay(SysCtlClockGet()/30);
    // Color mode //
    sendDisplayCommand(0x3A);
    SysCtlDelay(SysCtlClockGet()/30);
    // 16-bit color //
    sendDisplayData(0x05);
    SysCtlDelay(SysCtlClockGet()/30);
    // Display ON //
    sendDisplayCommand(0x29);
    SysCtlDelay(SysCtlClockGet()/30);

}

void resetDisplay(void){
    // RESET pin to low //
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
    // 10ms delay //
    SysCtlDelay(SysCtlClockGet()/300);
    // RESTE pin to high //
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    // 10ms delay //
    SysCtlDelay(SysCtlClockGet()/300);
}

void sendDisplayCommand(uint8_t cmd){
    // CS and DC to low (Enable the Display and put it in command mode) //
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0);
    // Send the command to the SPI3 //
    SSIDataPut(SSI3_BASE, cmd);
    // Wait for data to be tansfered //
    while(SSIBusy(SSI3_BASE));
    // CS high to disabble the display //
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, GPIO_PIN_1);
}

void sendDisplayData(uint8_t data){
    // CS low to enable the Display and DC high to data mode //
    GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1 | GPIO_PIN_2, 0x04);
    // Send the data to the SPI3 //
    SSIDataPut(SSI3_BASE, data);
    // Wait for data to be tansfered //
    while(SSIBusy(SSI3_BASE));
    // CS high to disabble the display //
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

void TFT_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    sendDisplayCommand(0x2A); // CASET
    sendDisplayData(((x0 + TFT_XSTART) >> 8) & 0xFF);
    sendDisplayData((x0 + TFT_XSTART) & 0xFF);
    sendDisplayData(((x1 + TFT_XSTART) >> 8) & 0xFF);
    sendDisplayData((x1 + TFT_XSTART) & 0xFF);

    sendDisplayCommand(0x2B); // RASET
    sendDisplayData(((y0 + TFT_YSTART) >> 8) & 0xFF);
    sendDisplayData((y0 + TFT_YSTART) & 0xFF);
    sendDisplayData(((y1 + TFT_YSTART) >> 8) & 0xFF);
    sendDisplayData((y1 + TFT_YSTART) & 0xFF);

    sendDisplayCommand(0x2C); // RAMWR
}


void TFT_FillScreen(uint16_t color)
{
    TFT_SetAddressWindow(0, 0, 127, 159);

    sendDisplayCommand(0x2C); // RAMWR
    uint32_t i;
    for (i = 0; i < (128UL * 160UL); i++)
    {
        sendDisplayData(color >> 8);
        sendDisplayData(color);
    }
}

void TFT_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
    if (x >= 128 || y >= 160) return;

    TFT_SetAddressWindow(x, y, x, y);
    sendDisplayData(color >> 8);
    sendDisplayData(color);
}

void TFT_DrawChar(int x, int y, char c, uint16_t color, uint16_t bg, uint8_t size)
{
    if (c < 32 || c > 126) c = '?';   // evitar índices fuera de rango
    uint8_t idx = (uint8_t)c - 32;    // IMPORTANTÍSIMO: ajustar al inicio de la tabla
    int col,bit,dx,dy,xx,yy;
    for (col = 0; col < 5; col++)
    {
        uint8_t line = font5x7[idx * 5 + col];

        // cada columna la expandimos verticalmente (8 bits)
        for (bit = 0; bit < 8; bit++)
        {
            bool pixelOn = (line >> bit) & 1;

            // Dibujar pixel(s) escalados 'size' veces
            if (pixelOn)
            {
                for (dx = 0; dx < size; dx++)
                    for (dy = 0; dy < size; dy++)
                        TFT_DrawPixel(x + col*size + dx, y + bit*size + dy, color);
            }
            else
            {
                // si quieres fondo transparente, comenta estas líneas:
                for (dx = 0; dx < size; dx++)
                    for (dy = 0; dy < size; dy++)
                        TFT_DrawPixel(x + col*size + dx, y + bit*size + dy, bg);
            }
        }
    }

    // columna de espacio (1 px) entre caracteres
    if (bg != color) { // si bg==color asumimos "transparente", opcional
        for (xx = 0; xx < size; xx++)
            for (yy = 0; yy < 8*size; yy++)
                TFT_DrawPixel(x + 5*size + xx, y + yy, bg);
    }
}

void TFT_DrawText(int x, int y, const char *text, uint16_t color, uint16_t bg, uint8_t size)
{
    while (*text)
    {
        TFT_DrawChar(x, y, *text, color, bg, size);
        x += 6 * size; // ancho de 5 + 1 px espacio
        text++;
    }
}

void TFT_DrawNumber(int x, int y, int num, uint16_t color, uint16_t bg, uint8_t size)
{
    char buffer[12];
    int i = 0,j;
    int isNegative = 0;

    if (num == 0) {
        buffer[i++] = '0';
        buffer[i] = '\0';
        TFT_DrawText(x, y, buffer, color, bg, size);
        return;
    }

    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    while (num > 0 && i < 11) {
        buffer[i++] = '0' + (num % 10);
        num /= 10;
    }

    if (isNegative) {
        buffer[i++] = '-';
    }

    buffer[i] = '\0';

    // invertir (la conversión quedó al revés)
    for (j = 0; j < i / 2; j++) {
        char tmp = buffer[j];
        buffer[j] = buffer[i - j - 1];
        buffer[i - j - 1] = tmp;
    }

    TFT_DrawText(x, y, buffer, color, bg, size);
}

