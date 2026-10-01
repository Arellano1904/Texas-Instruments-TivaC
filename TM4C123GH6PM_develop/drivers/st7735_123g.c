//*****************************************************************************
// 1.8inch_tft_display (ST7735, 128x160), tm4c123G, spi3_uDMA
//*****************************************************************************
// LIBRARIES
//*****************************************************************************
#include "st7735_123g.h"
// The inc folder contains the device header files for each TM4C device as well as the hardware header files.
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
// driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/udma.h"
// Own drivers
#include "delay.h"

//*****************************************************************************
// DEFINES
//*****************************************************************************
// SSI3 freq
#define SSI3_FRQ 20000000u

//*****************************************************************************
// GLOBALS VARIABLES
//*****************************************************************************
// Full ASCII 5x7 table (Adafruit font)
static const uint8_t font5x7[] = {
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
// System clock the SSI3 baud rate is derived from, set by st7735_init()
static uint32_t g_ui32SysClk = 0;
// For uDMA use
#pragma DATA_ALIGN(uDMAControlTable, 1024);
static uint8_t uDMAControlTable[1024];

//*****************************************************************************
// FUNCTION DEFINITIONS
//*****************************************************************************
// Peripheral config functions
void st7735_gpio_cnfg(void){ // Enable and wait for GPIO used by the display
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

void st7735_spi_cnfg(void){
    // Enable and wait for SSI peripheral ready
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI3));
    // Configure SSI3 module: Master mode, Motorola mode 0, 8bit len, SSI3_FRQ
    MAP_SSIDisable(SSI3_BASE);
    SSIClockSourceSet(SSI3_BASE,SSI_CLOCK_SYSTEM);
    MAP_SSIConfigSetExpClk(SSI3_BASE,g_ui32SysClk, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,SSI3_FRQ, 8);
    MAP_SSIEnable(SSI3_BASE);
}

void st7735_spi_len(uint8_t len){ // Change the SSI3 data len
    MAP_SSIDisable(SSI3_BASE);
    MAP_SSIConfigSetExpClk(SSI3_BASE,g_ui32SysClk, SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER,SSI3_FRQ, len);
    MAP_SSIEnable(SSI3_BASE);
}

void st7735_dma_cnfg(void){
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
    // Transfers are polled to completion in st7735_dma_snd_buffer(), so the
    // uDMA completion interrupt is left disabled on purpose (no ISR exists).

}

void st7735_dma_snd_buffer(uint16_t* buffer, uint32_t bufferLen){
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
void st7735_init(uint32_t ui32SysClock){
    // Init needed functions and hardware reset
    g_ui32SysClk = ui32SysClock;
    st7735_gpio_cnfg();
    delay_init(g_ui32SysClk);
    st7735_spi_cnfg();
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
    st7735_snd_dt(0x00);//180 rotation: MY=0, MX=0 (both axes flipped), MV=0 (still 128x160), RGB order (bit3=0)

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
    st7735_spi_len(16);
    // Arm the uDMA channel that streams RGB565 pixels to the SSI3 TX FIFO
    st7735_dma_cnfg();
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
void st7735_st_wndw(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1){
    // CASET/RASET/RAMWR are 8-bit frames: drop SSI3 back to 8 bits so each
    // byte clocks out as exactly one byte (16-bit mode would append a 0x00).
    st7735_spi_len(8);
    st7735_snd_cmd(ST7735_CASET);// column window
    st7735_snd_dt(0x00); st7735_snd_dt(x0);// start col
    st7735_snd_dt(0x00); st7735_snd_dt(x1);// end   col
    st7735_snd_cmd(ST7735_RASET);// row window
    st7735_snd_dt(0x00); st7735_snd_dt(y0);// start row
    st7735_snd_dt(0x00); st7735_snd_dt(y1);// end   row
    st7735_snd_cmd(ST7735_RAMWR);// the pixels that follow land in this window
    // Pixels stream as 16-bit RGB565 frames, so go back to 16-bit len
    st7735_spi_len(16);
}
void st7735_fll_scrn(uint16_t color){
    // One uDMA basic transfer moves at most 1024 items, so keep a 1024-pixel
    // colour chunk and re-send it until the whole panel is painted (that is the
    // most pixels a single DMA burst can push).
    static uint16_t chunk[1024];
    uint32_t i;
    for(i = 0; i < 1024; i++) chunk[i] = color;
    // Whole panel is the target window
    st7735_st_wndw(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1);
    // Push in <=1024-pixel DMA bursts; CS toggles per burst but no command is
    // issued in between, so the ST7735 keeps writing GRAM as one continuous run.
    uint32_t pixels = (uint32_t)ST7735_WIDTH * ST7735_HEIGHT;
    while(pixels){
        uint32_t n = (pixels > 1024) ? 1024 : pixels;
        st7735_dma_snd_buffer(chunk, n);
        pixels -= n;
    }
}
void st7735_drw_chr(uint8_t x, uint8_t y, char c, uint16_t color, uint16_t bg){
    // Only printable ASCII 32..126 lives in the font table; map the rest to '?'
    if(c < 32 || c > 126) c = '?';
    const uint8_t *glyph = &font5x7[(c - 32) * 5];
    // Build the whole 6x8 cell (5 glyph columns + 1 spacing, 8 rows) row-major
    // so it matches the RAMWR scan order (X advances first, then Y), then push
    // all 48 pixels in a single DMA burst.
    uint16_t cell[6 * 8];
    uint8_t row, col;
    for(row = 0; row < 8; row++){
        for(col = 0; col < 6; col++){
            uint8_t on = (col < 5) ? ((glyph[col] >> row) & 0x01) : 0;// 6th col = gap
            cell[row * 6 + col] = on ? color : bg;
        }
    }
    st7735_st_wndw(x, y, x + 5, y + 7);
    st7735_dma_snd_buffer(cell, 6 * 8);
}
void st7735_prtn_str(uint8_t x, uint8_t y, const char* str, uint16_t color, uint16_t bg){
    while(*str){
        // Wrap to the next text line when the glyph would run off the right edge
        if(x + 6 > ST7735_WIDTH){ x = 0; y += 8; }
        // Stop once we run past the bottom of the panel
        if(y + 8 > ST7735_HEIGHT) break;
        st7735_drw_chr(x, y, *str++, color, bg);
        x += 6;// 5 glyph columns + 1 spacing column
    }
}
void st7735_prtn_int(uint8_t x, uint8_t y, int32_t value, uint16_t color, uint16_t bg){
    char buf[12];// fits -2147483648 plus the null terminator
    char tmp[12];
    uint8_t i = 0, t = 0;
    bool neg = (value < 0);
    // Magnitude taken the INT_MIN-safe way (negating INT_MIN would overflow)
    uint32_t mag = neg ? ((uint32_t)(-(value + 1)) + 1u) : (uint32_t)value;
    // Extract digits low-to-high, then reverse them into the print buffer
    do{ tmp[t++] = (char)('0' + (mag % 10)); mag /= 10; }while(mag);
    if(neg) buf[i++] = '-';
    while(t) buf[i++] = tmp[--t];
    buf[i] = '\0';
    st7735_prtn_str(x, y, buf, color, bg);
}
void st7735_prtn_float(uint8_t x, uint8_t y, float value, uint8_t decimals, uint16_t color, uint16_t bg){
    char buf[20];
    char tmp[12];
    uint8_t i = 0, t = 0;
    if(value < 0.0f){ buf[i++] = '-'; value = -value; }
    // Integer part first (reverse-fill then flip), then the requested decimals
    uint32_t ip = (uint32_t)value;
    float frac = value - (float)ip;
    do{ tmp[t++] = (char)('0' + (ip % 10)); ip /= 10; }while(ip);
    while(t) buf[i++] = tmp[--t];
    if(decimals){
        buf[i++] = '.';
        // Truncating conversion (no rounding), one digit at a time
        while(decimals--){
            frac *= 10.0f;
            uint8_t d = (uint8_t)frac;
            buf[i++] = (char)('0' + d);
            frac -= (float)d;
        }
    }
    buf[i] = '\0';
    st7735_prtn_str(x, y, buf, color, bg);
}
