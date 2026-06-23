// C file to handle a 1.8_st7735_spi_display
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
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"
#include "driverlib/systick.h"
// The inc folder contains the device header files for each TM4C device as well as the hardware header files.
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
#include "inc/hw_udma.h"
#include "inc/hw_nvic.h"
// Own driver libraries
#include "drivers/1.8_st7735_display.h"
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

// Useful defines
#define st7735_WIDTH   128
#define st7735_HEIGHT  160

// Font / character-cell geometry (Adafruit 5x7 glyph + 1 px spacing).
#define FONT_WIDTH     5            // glyph columns read from Font5x7
#define CHAR_WIDTH     6            // glyph + inter-character spacing column
#define CHAR_HEIGHT    8            // glyph rows + spacing row

// The uDMA controller moves at most 1024 items in a single basic-mode request.
#define UDMA_MAX_ITEMS     1024
// Whole characters that fit in one DMA request (21 * 48 = 1008 <= 1024).
#define MAX_CHARS_PER_DMA  (UDMA_MAX_ITEMS / (CHAR_WIDTH * CHAR_HEIGHT))

// Array for uDMA requirement.
#pragma DATA_ALIGN(uDMAControlTable, 1024);
uint8_t uDMAControlTable[1024];

// Shared 16-bit pixel buffer, source for every pixel DMA transfer (screen
// fills and rendered text). Sized to the uDMA single-request maximum.
static uint16_t dmaBuffer[UDMA_MAX_ITEMS];

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

    // Configure the SSI3 as SPI MODE0, 16-bit frames, 30 MHz bit clock //
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
        UDMA_SIZE_16 |        // 16-bit data (one RGB565 pixel per item)
        UDMA_SRC_INC_16 |     // walk through the source buffer
        UDMA_DST_INC_NONE |   // SSI3 data register stays put
        UDMA_ARB_8);          // re-arbitrate every 8 items (SSI FIFO depth)

    // Transfers are polled to completion in uDMA_spi3_send_buffer(), so the
    // uDMA completion interrupt is left disabled on purpose (no ISR exists).
}

void uDMA_spi3_send_buffer(uint16_t* dataBuffer, uint32_t count){
    // CS low to enable the Display and DC high to data mode //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1 | GPIO_PIN_2, 0x04);
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
    // Wait for completion
    while(MAP_uDMAChannelIsEnabled(UDMA_CH15_SSI3TX));
    // Wait for SSI to finish shifting last word
    while(MAP_SSIBusy(SSI3_BASE));

    // CS high to disabble the display //
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

//*****************************************************************************
// Blocking delays built on the Cortex-M SysTick down-counter.
//
// SysTick is a dedicated 24-bit core timer, so these delays are clock-accurate
// (paced by the system clock) and consume no general-purpose timer. SysTick is
// unused elsewhere and only runs here during display init/reset, so blocking on
// it is safe. Replaces the approximate, hand-tuned MAP_SysCtlDelay() loops
// (which were also mistimed: the init "10 ms" waits were really ~100 ms).
//*****************************************************************************
static void delay_us(uint32_t us){
    const uint32_t ticksPerUs = MAP_SysCtlClockGet() / 1000000U;  // 80 @ 80 MHz
    const uint32_t maxChunkUs = 100000U;                          // keeps reload < 2^24
    while(us){
        uint32_t chunk = (us > maxChunkUs) ? maxChunkUs : us;
        MAP_SysTickPeriodSet(chunk * ticksPerUs);
        HWREG(NVIC_ST_CURRENT) = 0;   // any write clears the counter and COUNTFLAG
        MAP_SysTickEnable();
        // Spin until the counter reloads (COUNTFLAG, bit 16, asserts).
        while((HWREG(NVIC_ST_CTRL) & NVIC_ST_CTRL_COUNT) == 0){ }
        MAP_SysTickDisable();
        us -= chunk;
    }
}

static void delay_ms(uint32_t ms){
    while(ms--)
        delay_us(1000U);
}

//*****************************************************************************
// Power-on initialization sequence (ST7735R).
//
// Data-driven table: each row is { command, parameter count, parameters,
// post-command delay (ms) }. The frame-rate / power / VCOM / gamma values are
// the manufacturer-recommended ST7735R settings; together with the existing
// 30 MHz SPI + uDMA pixel path they give the best image quality and the fastest
// refresh the panel reliably supports. MADCTL is left at 0x00 so the orientation
// and RGB colour order match the previous (working) configuration.
//*****************************************************************************
typedef struct {
    uint8_t  cmd;
    uint8_t  numArgs;
    uint8_t  args[16];
    uint16_t delayMs;
} ST7735_InitCmd;

static const ST7735_InitCmd st7735_init_seq[] = {
    { 0x01, 0, { 0 },                           150 }, // SWRESET: software reset
    { 0x11, 0, { 0 },                           150 }, // SLPOUT: exit sleep, let booster/clocks settle
    // Frame-rate control. Lower porches -> higher refresh. 0x00,0x06,0x03 is the
    // fastest setting (~125 Hz, best performance). If you see flicker or banding,
    // fall back to the conservative 0x01,0x2C,0x2D (~80 Hz).
    { 0xB1, 3, { 0x00, 0x06, 0x03 },              0 }, // FRMCTR1: frame rate, normal mode
    { 0xB2, 3, { 0x01, 0x2C, 0x2D },              0 }, // FRMCTR2: frame rate, idle mode
    { 0xB3, 6, { 0x01, 0x2C, 0x2D,
                 0x01, 0x2C, 0x2D },              0 }, // FRMCTR3: frame rate, partial mode
    { 0xB4, 1, { 0x07 },                          0 }, // INVCTR: no display inversion
    { 0xC0, 3, { 0xA2, 0x02, 0x84 },              0 }, // PWCTR1: power control
    { 0xC1, 1, { 0xC5 },                          0 }, // PWCTR2: power control
    { 0xC2, 2, { 0x0A, 0x00 },                    0 }, // PWCTR3: power control (normal mode)
    { 0xC3, 2, { 0x8A, 0x2A },                    0 }, // PWCTR4: power control (idle mode)
    { 0xC4, 2, { 0x8A, 0xEE },                    0 }, // PWCTR5: power control (partial mode)
    { 0xC5, 1, { 0x0E },                          0 }, // VMCTR1: VCOM voltage
    { 0x20, 0, { 0 },                             0 }, // INVOFF: inversion off
    { 0x36, 1, { 0x00 },                          0 }, // MADCTL: RGB order, native orientation
    { 0x3A, 1, { 0x05 },                          0 }, // COLMOD: 16 bpp (RGB565)
    { 0xE0, 16, { 0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
                  0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10 }, 0 }, // GMCTRP1: positive gamma
    { 0xE1, 16, { 0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                  0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10 }, 0 }, // GMCTRN1: negative gamma
    { 0x13, 0, { 0 },                            10 }, // NORON: normal display mode on
    { 0x29, 0, { 0 },                           100 }, // DISPON: display on
};

// Display
void st7735_init(void){
    // Hardware reset (includes the required reset-pulse delays).
    st7735_reset();

    // Commands and their parameters are single bytes.
    spi3_len_config(8);

    uint32_t i;
    uint8_t  a;
    for(i = 0; i < sizeof(st7735_init_seq) / sizeof(st7735_init_seq[0]); i++){
        const ST7735_InitCmd *c = &st7735_init_seq[i];
        st7735_send_command(c->cmd);
        for(a = 0; a < c->numArgs; a++)
            st7735_send_data(c->args[a]);
        if(c->delayMs)
            delay_ms(c->delayMs);
    }

    // Switch to 16-bit frames for fast pixel streaming via uDMA.
    spi3_len_config(16);
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
    MAP_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, GPIO_PIN_1);
}

void st7735_reset(void){
    // RESET pin low: assert reset (datasheet min pulse 10 us; 20 ms is generous).
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
    delay_ms(20);
    // RESET pin high: release reset, then wait for the controller to come up.
    MAP_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
    delay_ms(120);
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

void st7735_fill_screen(uint16_t color){
    uint32_t remaining = (uint32_t)st7735_WIDTH * st7735_HEIGHT;
    uint32_t i;

    // Pre-load the whole DMA buffer with the fill color once.
    for(i = 0; i < UDMA_MAX_ITEMS; i++)
        dmaBuffer[i] = color;

    // Address the full panel, then stream it in maximum-size DMA bursts.
    st7735_set_window(0, 0, st7735_WIDTH - 1, st7735_HEIGHT - 1);

    while(remaining){
        uint32_t chunk = (remaining > UDMA_MAX_ITEMS) ? UDMA_MAX_ITEMS : remaining;
        uDMA_spi3_send_buffer(dmaBuffer, chunk);
        remaining -= chunk;
    }
}

// Render a run of up to MAX_CHARS_PER_DMA characters into the shared buffer and
// push the whole block to the panel in a SINGLE uDMA request. The buffer is
// laid out row-major to match the order the controller writes pixels into the
// addressed window: row 0 of every character, then row 1, and so on.
static void st7735_draw_run(uint16_t x, uint16_t y, const char *str, uint8_t n,
                            uint16_t fg, uint16_t bg){
    uint16_t runWidth = (uint16_t)n * CHAR_WIDTH;
    uint8_t row, col, ch;

    for(row = 0; row < CHAR_HEIGHT; row++)
    {
        uint16_t rowBase = (uint16_t)row * runWidth;

        for(ch = 0; ch < n; ch++)
        {
            char c = str[ch];
            const uint8_t *glyph;
            uint16_t base = rowBase + (uint16_t)ch * CHAR_WIDTH;

            if(c < 32 || c > 126)
                c = '?';
            glyph = &Font5x7[(c - 32) * FONT_WIDTH];

            for(col = 0; col < FONT_WIDTH; col++)
                dmaBuffer[base + col] = (glyph[col] & (1 << row)) ? fg : bg;

            // inter-character spacing column
            dmaBuffer[base + FONT_WIDTH] = bg;
        }
    }

    // One window, one transfer for the whole run.
    st7735_set_window(x, y, x + runWidth - 1, y + CHAR_HEIGHT - 1);
    uDMA_spi3_send_buffer(dmaBuffer, (uint32_t)runWidth * CHAR_HEIGHT);
}

void st7735_draw_char(uint16_t x,uint16_t y,char c,uint16_t fg,uint16_t bg){
    char s[2];
    s[0] = c;
    s[1] = '\0';
    st7735_draw_run(x, y, s, 1, fg, bg);
}

void st7735_print_string(uint16_t x,uint16_t y,const char *str,uint16_t fg,uint16_t bg){
    while(*str)
    {
        uint8_t charsToEdge, maxRun, run;

        // Whole characters that still fit on the current line.
        charsToEdge = (st7735_WIDTH - x) / CHAR_WIDTH;
        if(charsToEdge == 0)
        {
            // Wrap to the next text line.
            x = 0;
            y += CHAR_HEIGHT;
            charsToEdge = st7735_WIDTH / CHAR_WIDTH;
        }

        // Cap the run by what fits on the line and by one DMA request.
        maxRun = (charsToEdge < MAX_CHARS_PER_DMA) ? charsToEdge : MAX_CHARS_PER_DMA;

        run = 0;
        while(run < maxRun && str[run])
            run++;

        st7735_draw_run(x, y, str, run, fg, bg);

        str += run;
        x   += (uint16_t)run * CHAR_WIDTH;
    }
}

void int_to_str(int32_t value, char *buf)
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

void float_to_str(float value, char *buf, uint8_t decimals)
{
    int32_t intPart;
    float frac, rounding;
    char intBuf[12];
    int i = 0, j;
    uint8_t d;
    int negative = 0;

    // NaN / Inf produce garbage when cast to int, so report them explicitly.
    if (value != value) { buf[0] = 'n'; buf[1] = 'a'; buf[2] = 'n'; buf[3] = '\0'; return; }
    if (value >  3.4e38f || value < -3.4e38f)
    {
        if (value < 0) buf[i++] = '-';
        buf[i++] = 'i'; buf[i++] = 'n'; buf[i++] = 'f'; buf[i] = '\0';
        return;
    }

    if (value < 0.0f)
    {
        negative = 1;
        value = -value;
    }

    // Round half away from zero: add half of the last shown digit's weight.
    rounding = 0.5f;
    for (d = 0; d < decimals; d++)
        rounding *= 0.1f;

    // Only emit the sign if the value still rounds to something non-zero,
    // so e.g. -0.001 at 2 decimals prints "0.00" instead of "-0.00".
    if (negative && value >= rounding)
        buf[i++] = '-';

    value += rounding;            // rounding may carry into the integer part

    intPart = (int32_t)value;
    frac = value - (float)intPart;

    // Integer part
    int_to_str(intPart, intBuf);
    for (j = 0; intBuf[j]; j++)
        buf[i++] = intBuf[j];

    // Fractional part (omit the dot entirely when no decimals are requested)
    if (decimals > 0)
    {
        buf[i++] = '.';
        while (decimals--)
        {
            int digit;
            frac *= 10.0f;
            digit = (int)frac;
            if (digit > 9) digit = 9;   // guard against float rounding error
            buf[i++] = (char)(digit + '0');
            frac -= digit;
        }
    }

    buf[i] = '\0';
}

void st7735_print_int(uint16_t x, uint16_t y,int32_t value,uint16_t color, uint16_t bg)
{
    char buf[12];
    int_to_str(value, buf);
    st7735_print_string(x, y, buf, color, bg);
}

void st7735_print_float(uint16_t x, uint16_t y,float value, uint8_t decimals,uint16_t color, uint16_t bg)
{
    char buf[20];
    float_to_str(value, buf, decimals);
    st7735_print_string(x, y, buf, color, bg);
}

