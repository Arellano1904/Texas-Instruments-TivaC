//*****************************************************************************
// xpt2046 driver for 2.4_tft_display, ILI9341_driver, tm4c1294_spi2_uDMA
//*****************************************************************************
// xpt2046_tm4c Touch Driver
#include "xpt2046_tm4c1294.h"
// Common used libraries.
#include <stdio.h>
#include <stdbool.h>
// The inc folder contains the device header files for each TM4C device as well as the hardware header.
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ssi.h"
// The driverlib folder contains the TivaWare Driver Library (DriverLib) source code that allows users to leverage TI validated functions.
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
//*****************************************************************************
// //***** Definitions *****//
//*****************************************************************************

//*****************************************************************************
// //***** Functions definitions *****//
//*****************************************************************************