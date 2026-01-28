//*****************************************************************************
// Main code to test functionalities
//*****************************************************************************

//*****************************************************************************
// Libraries
//*****************************************************************************
// Common use libraries
#include <stdint.h>
#include <stdbool.h>
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions. 
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"
// The inc folder contains the device header files for each TM4C device as well as the hardware header files.
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"

//*****************************************************************************
// Useful definitions.
//*****************************************************************************

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
#define st7735_WIDTH   128
#define st7735_HEIGHT  160

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
// Array for uDMA requirement.
#pragma DATA_ALIGN(uDMAControlTable, 1024);
uint8_t uDMAControlTable[1024];

//***** Flags variables *****//
uint8_t sw1State, sw2State = 0x00;
volatile bool spi3_dma_done = false;

// Buffer to fill a color line  //
uint16_t colorBuffer[st7735_WIDTH];

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
// Function declaration.
//*****************************************************************************

// On board buttons.
void buttons_config(void);
void buttons_handler(void);

// On board rgbLed
void rgb_led_config(void);

// SPI3-uDMA
void spi3_config(void);
void spi3_len_config(uint16_t bits);

// uDMA
void uDMA_spi3_config(void);
void uDMA_spi3_send_buffer(uint16_t* dataBuffer, uint32_t count);
void uDMA_spi3_int_handler(void);

// Display
void st7735_init();
void st7735_send_command(uint8_t cmd);
void st7735_send_data(uint8_t data);
void st7735_reset(void);
void st7735_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1);
void st7735_fill_screen(uint16_t color);

//*****************************************************************************
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//*****************************************************************************
int main(void)
{
    // Setup the system clock to run at 80 Mhz from PLL with crystal reference
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
    
    // Floating point unit enabling //
    MAP_FPUEnable();
    MAP_FPULazyStackingEnable();
    
    // Invoque configuration functions
    buttons_config();
    rgb_led_config();
    spi3_config();
    uDMA_spi3_config();

    // Enable interrupts globally
    MAP_IntMasterEnable();

    // Init display module
    st7735_init();
    // Fille the screen
    st7735_fill_screen(BLACK);
    // Loop Forever
    while(1)
    {
        if(sw1State){ // SW1 pressed
            sw1State = 0x0;
            // Turn red led on.
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x02);
            // Fille the screen
            st7735_fill_screen(RED);
        }
        if(sw2State){// SW2 pressed
            sw2State = 0x00;
            // Turn blue led on.
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x04);
            // Fille the screen
            st7735_fill_screen(BLUE);
        }
    }
}

//*****************************************************************************
// Function definitions.
//*****************************************************************************
// On board buttons :
void buttons_config(void){
    // Enable and wait to be ready for GPIO port F.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // ===== UNLOCK PF0 (SW2) =====
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;   // Unlock key
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR)  |= GPIO_PIN_0;   // Commit
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
    // Configure PF0 and PF4 as inputs.
    MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // 3. Enable internal pull-up resistor (for active-low buttons)
    MAP_GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
    // 1. Set the interrupt type (e.g., falling edge). GPIO_FALLING_EDGE is standard for an active-low button press
    MAP_GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_FALLING_EDGE);
    // 2. Clear any pending interrupt flags for the pins. This is critical to prevent an immediate, false trigger
    MAP_GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // 3. Unmask the interrupt for the specific pins.This enables the pin to generate an interrupt
    MAP_GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
    // 4. Enable the GPIO Port F interrupt in the NVIC
    MAP_IntEnable(INT_GPIOF);
}
void buttons_handler(void){
    // Read and clear the interrupt status
    uint32_t ui32Status = MAP_GPIOIntStatus(GPIO_PORTF_BASE, true);
    
    if (ui32Status & GPIO_PIN_0) // Switch 2
    {    
        sw2State = 0x01;
        // Clears the interrupt triger
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0);
    }
    if (ui32Status & GPIO_PIN_4) // Switch 1
    {
        sw1State = 0x01;
        // Clears the interrupt triger
        GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);
    }
}

// On board rgbLed
void rgb_led_config(void){
    // Enable and wait to be ready for GPIO port F.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    // Set PF1, PF2 and PF3 as output.
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 ); 
}
// SPI3 
void spi3_config(void){
    // Enable SSI3, PORT_D and PORT_B //
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // Configure GPIO Port D pins 0 and 3 to be used as SSI
    MAP_GPIOPinTypeSSI(GPIO_PORTD_BASE,GPIO_PIN_0 | GPIO_PIN_3);

    // Configure PD0, PD3 for SSI3 (CLK, TX) //
    MAP_GPIOPinConfigure(GPIO_PD0_SSI3CLK); 
    MAP_GPIOPinConfigure(GPIO_PD3_SSI3TX);

    // Configure ChipSelect (CS-PD1) and DataComand (DC-PD2) pins  //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2);

    // Configure RESET pin (PB5) //
    MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,  GPIO_PIN_5);

    // Configure the SSI3 as SPI MODE0, 8-bit, 16 MHz // 
    MAP_SSIConfigSetExpClk(SSI3_BASE,MAP_SysCtlClockGet(),SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,30000000,16);
    // Enabling the SSI3 module //
    MAP_SSIEnable(SSI3_BASE);
}

void spi3_len_config(uint16_t bits){
    // Disable QSSI3
    MAP_SSIDisable(SSI3_BASE);
    // Configure the bits len
    MAP_SSIConfigSetExpClk(SSI3_BASE, MAP_SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,30000000, bits);
    // Enable QSSI3
    MAP_SSIEnable(SSI3_BASE);
}

void uDMA_spi3_config(void){
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
        UDMA_SIZE_16 |        // 16-bit data
        UDMA_SRC_INC_16 |     // increment source
        UDMA_DST_INC_NONE |   // SSI FIFO
        UDMA_ARB_8);          // burst of 16 words

    // Ennabling the interruption
    MAP_IntEnable(INT_UDMA);
}

void uDMA_spi3_send_buffer(uint16_t* dataBuffer, uint32_t count){
    // Disable channel before setup
    MAP_uDMAChannelDisable(UDMA_CH15_SSI3TX);

    // Configure transfer
    MAP_uDMAChannelTransferSet(
       UDMA_CH15_SSI3TX | UDMA_PRI_SELECT,
        UDMA_MODE_BASIC,
        dataBuffer,
        (void *)(SSI3_BASE + SSI_O_DR),
        count
    );

    // Start transfer
    MAP_uDMAChannelEnable(UDMA_CH15_SSI3TX);
}

// Display 
void st7735_init(){
    // No coments needed //
    st7735_reset();
    // Config the spi3 to send commands and data configuration.
    spi3_len_config(8);
    // Software reset //
    st7735_send_command(0x01); 
    // 10ms delay to wait for the display response //
    MAP_SysCtlDelay(MAP_SysCtlClockGet()/30);
    // Sleep out //
    st7735_send_command(0x11);
    MAP_SysCtlDelay(MAP_SysCtlClockGet()/30);
    // Color mode //
    st7735_send_command(0x3A);
    MAP_SysCtlDelay(MAP_SysCtlClockGet()/30);
    // 16-bit color //
    st7735_send_data(0x05);
    MAP_SysCtlDelay(MAP_SysCtlClockGet()/30);
    // Display ON //
    st7735_send_command(0x29);
    // Config the spi3 to send pixels data.
    spi3_len_config(16);
    MAP_SysCtlDelay(MAP_SysCtlClockGet()/30);
}

void st7735_send_command(uint8_t cmd){
    // CS and DC to low (Enable the Display and put it in command mode) //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0);
    // Send the command to the SPI3 //
    MAP_SSIDataPut(SSI3_BASE, cmd);
    // Wait for data to be tansfered //
    while(MAP_SSIBusy(SSI3_BASE));
    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, GPIO_PIN_1);
}

void st7735_send_data(uint8_t data){
    // CS low to enable the Display and DC high to data mode //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1 | GPIO_PIN_2, 0x04);
    // Send the data to the SPI3 //
    MAP_SSIDataPut(SSI3_BASE, data);
    // Wait for data to be tansfered //
    while(MAP_SSIBusy(SSI3_BASE));
    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);;
}

void st7735_reset(void){
    // RESET pin to low //
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
    // 10ms delay //
    MAP_SysCtlDelay(MAP_SysCtlClockGet()/300);
    // RESTE pin to high //
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    // 10ms delay //
    MAP_SysCtlDelay(MAP_SysCtlClockGet()/300);
}

void st7735_set_window(uint16_t x0, uint16_t y0,uint16_t x1, uint16_t y1){
    // Config spi3 to send command and data configuration
    spi3_len_config(8);
    // Column Address Set (CASET)
    st7735_send_command(0x2A);
    st7735_send_data(x0 >> 8); st7735_send_data(x0);
    st7735_send_data(x1 >> 8); st7735_send_data(x1);

    st7735_send_command(0x2B);
    st7735_send_data(y0 >> 8); st7735_send_data(y0);
    st7735_send_data(y1 >> 8); st7735_send_data(y1);

    // Write to RAM
    st7735_send_command(0x2C);
    // Config spi3 to send pixels data
    spi3_len_config(16);
    
}

void st7735_fill_screen(uint16_t color){ // Aqui en fill screen ha de estar el pedo.
    uint32_t i;
    // Prepare one line
    for(i = 0; i < st7735_WIDTH; i++)
        colorBuffer[i] = color;
    // Set full window
    st7735_set_window(0, 0,st7735_WIDTH - 1,st7735_HEIGHT - 1);
    // CS low, DC high (DATA mode)
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_2, 0x04);
    // Stream lines using uDMA
    for(i = 0; i < st7735_HEIGHT; i++){ 
        uDMA_spi3_send_buffer(colorBuffer, st7735_WIDTH); // Esta funcion ya no espera a que se termine la transmision.
        // Wait for completion
        while(MAP_uDMAChannelIsEnabled(UDMA_CH15_SSI3TX));

        // Wait for SSI to finish shifting last word
        while(MAP_SSIBusy(SSI3_BASE));
    }

    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1, GPIO_PIN_1);
}

