/*
 * Copyright (c) 2015-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ====================== CC1310_LAUNCHXL.c ===================================
 *  This file is responsible for setting up the board specific items for the
 *  CC1310_LAUNCHXL board.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/cc13x0/driverlib/ioc.h>
#include <ti/devices/cc13x0/driverlib/udma.h>
#include <ti/devices/cc13x0/inc/hw_ints.h>
#include <ti/devices/cc13x0/inc/hw_memmap.h>

#include "CC1310_LAUNCHXL.h"

/*
 *  =============================== ADCBuf ===============================
 */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufCC26XX.h>

ADCBufCC26XX_Object adcBufCC26xxObjects[CC1310_LAUNCHXL_ADCBUFCOUNT];

/*
 *  This table converts a virtual adc channel into a dio and internal analogue
 *  input signal. This table is necessary for the functioning of the adcBuf
 *  driver. Comment out unused entries to save flash. Dio and internal signal
 *  pairs are hardwired. Do not remap them in the table. You may reorder entire
 *  entries. The mapping of dio and internal signals is package dependent.
 */
const ADCBufCC26XX_AdcChannelLutEntry ADCBufCC26XX_adcChannelLut[CC1310_LAUNCHXL_ADCBUF0CHANNELCOUNT] = {
    {CC1310_LAUNCHXL_DIO23_ANALOG, ADC_COMPB_IN_AUXIO7},
    {CC1310_LAUNCHXL_DIO24_ANALOG, ADC_COMPB_IN_AUXIO6},
    {CC1310_LAUNCHXL_DIO25_ANALOG, ADC_COMPB_IN_AUXIO5},
    {CC1310_LAUNCHXL_DIO26_ANALOG, ADC_COMPB_IN_AUXIO4},
    {CC1310_LAUNCHXL_DIO27_ANALOG, ADC_COMPB_IN_AUXIO3},
    {CC1310_LAUNCHXL_DIO28_ANALOG, ADC_COMPB_IN_AUXIO2},
    {CC1310_LAUNCHXL_DIO29_ANALOG, ADC_COMPB_IN_AUXIO1},
    {CC1310_LAUNCHXL_DIO30_ANALOG, ADC_COMPB_IN_AUXIO0},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VDDS},
    {PIN_UNASSIGNED, ADC_COMPB_IN_DCOUPL},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VSS},
};

const ADCBufCC26XX_HWAttrs adcBufCC26xxHWAttrs[CC1310_LAUNCHXL_ADCBUFCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
        .adcChannelLut     = ADCBufCC26XX_adcChannelLut,
        .gpTimerUnit       = CC1310_LAUNCHXL_GPTIMER0A,
        .gptDMAChannelMask = 1 << UDMA_CHAN_TIMER0_A,
    }
};

const ADCBuf_Config ADCBuf_config[CC1310_LAUNCHXL_ADCBUFCOUNT] = {
    {
        &ADCBufCC26XX_fxnTable,
        &adcBufCC26xxObjects[CC1310_LAUNCHXL_ADCBUF0],
        &adcBufCC26xxHWAttrs[CC1310_LAUNCHXL_ADCBUF0]
    },
};

const uint_least8_t ADCBuf_count = CC1310_LAUNCHXL_ADCBUFCOUNT;

/*
 *  =============================== ADC ===============================
 */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>

ADCCC26XX_Object adcCC26xxObjects[CC1310_LAUNCHXL_ADCCOUNT];


const ADCCC26XX_HWAttrs adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCCOUNT] = {
    {
        .adcDIO              = CC1310_LAUNCHXL_DIO23_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO7,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1310_LAUNCHXL_DIO24_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO6,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1310_LAUNCHXL_DIO25_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO5,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1310_LAUNCHXL_DIO26_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO4,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1310_LAUNCHXL_DIO27_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO3,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1310_LAUNCHXL_DIO28_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO2,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1310_LAUNCHXL_DIO29_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO1,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = CC1310_LAUNCHXL_DIO30_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO0,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_10P9_MS,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_DCOUPL,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_VSS,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = PIN_UNASSIGNED,
        .adcCompBInput       = ADC_COMPB_IN_VDDS,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    }
};

const ADC_Config ADC_config[CC1310_LAUNCHXL_ADCCOUNT] = {
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC0], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC0]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC1], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC1]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC2], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC2]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC3], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC3]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC4], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC4]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC5], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC5]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC6], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC6]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADC7], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADC7]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADCDCOUPL], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCDCOUPL]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADCVSS], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCVSS]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[CC1310_LAUNCHXL_ADCVDDS], &adcCC26xxHWAttrs[CC1310_LAUNCHXL_ADCVDDS]},
};

const uint_least8_t ADC_count = CC1310_LAUNCHXL_ADCCOUNT;

/*
 *  =============================== Crypto ===============================
 */
#include <ti/drivers/crypto/CryptoCC26XX.h>

CryptoCC26XX_Object cryptoCC26XXObjects[CC1310_LAUNCHXL_CRYPTOCOUNT];

const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC1310_LAUNCHXL_CRYPTOCOUNT] = {
    {
        .baseAddr       = CRYPTO_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_CRYPTO,
        .intNum         = INT_CRYPTO_RESULT_AVAIL_IRQ,
        .intPriority    = ~0,
    }
};

const CryptoCC26XX_Config CryptoCC26XX_config[CC1310_LAUNCHXL_CRYPTOCOUNT] = {
    {
         .object  = &cryptoCC26XXObjects[CC1310_LAUNCHXL_CRYPTO0],
         .hwAttrs = &cryptoCC26XXHWAttrs[CC1310_LAUNCHXL_CRYPTO0]
    }
};

/*
 *  =============================== Display ===============================
 */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/display/DisplaySharp.h>

#ifndef BOARD_DISPLAY_UART_STRBUF_SIZE
#define BOARD_DISPLAY_UART_STRBUF_SIZE    128
#endif

#ifndef BOARD_DISPLAY_SHARP_SIZE
#define BOARD_DISPLAY_SHARP_SIZE    96
#endif

DisplayUart_Object     displayUartObject;
DisplaySharp_Object    displaySharpObject;

static char uartStringBuf[BOARD_DISPLAY_UART_STRBUF_SIZE];
static uint_least8_t sharpDisplayBuf[BOARD_DISPLAY_SHARP_SIZE * BOARD_DISPLAY_SHARP_SIZE / 8];

const DisplayUart_HWAttrs displayUartHWAttrs = {
    .uartIdx      = CC1310_LAUNCHXL_UART0,
    .baudRate     = 115200,
    .mutexTimeout = (unsigned int)(-1),
    .strBuf       = uartStringBuf,
    .strBufLen    = BOARD_DISPLAY_UART_STRBUF_SIZE,
};

const DisplaySharp_HWAttrs displaySharpHWattrs = {
    .spiIndex    = CC1310_LAUNCHXL_SPI0,
    .csPin       = CC1310_LAUNCHXL_LCD_CS,
    .extcominPin = CC1310_LAUNCHXL_LCD_EXTCOMIN,
    .powerPin    = CC1310_LAUNCHXL_LCD_POWER,
    .enablePin   = CC1310_LAUNCHXL_LCD_ENABLE,
    .pixelWidth  = BOARD_DISPLAY_SHARP_SIZE,
    .pixelHeight = BOARD_DISPLAY_SHARP_SIZE,
    .displayBuf  = sharpDisplayBuf,
};

#ifndef BOARD_DISPLAY_USE_UART
#define BOARD_DISPLAY_USE_UART 1
#endif
#ifndef BOARD_DISPLAY_USE_UART_ANSI
#define BOARD_DISPLAY_USE_UART_ANSI 0
#endif
#ifndef BOARD_DISPLAY_USE_LCD
#define BOARD_DISPLAY_USE_LCD 0
#endif

/*
 * This #if/#else is needed to workaround a problem with the
 * IAR compiler. The IAR compiler doesn't like the empty array
 * initialization. (IAR Error[Pe1345])
 */
#if (BOARD_DISPLAY_USE_UART || BOARD_DISPLAY_USE_LCD)

const Display_Config Display_config[] = {
#if (BOARD_DISPLAY_USE_UART)
    {
#  if (BOARD_DISPLAY_USE_UART_ANSI)
        .fxnTablePtr = &DisplayUartAnsi_fxnTable,
#  else /* Default to minimal UART with no cursor placement */
        .fxnTablePtr = &DisplayUartMin_fxnTable,
#  endif
        .object      = &displayUartObject,
        .hwAttrs     = &displayUartHWAttrs,
    },
#endif
#if (BOARD_DISPLAY_USE_LCD)
    {
        .fxnTablePtr = &DisplaySharp_fxnTable,
        .object      = &displaySharpObject,
        .hwAttrs     = &displaySharpHWattrs
    },
#endif
};

const uint_least8_t Display_count = sizeof(Display_config) / sizeof(Display_Config);

#else

const Display_Config *Display_config = NULL;
const uint_least8_t Display_count = 0;

#endif /* (BOARD_DISPLAY_USE_UART || BOARD_DISPLAY_USE_LCD) */

/*
 *  =============================== GPIO ===============================
 */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOCC26XX.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC1310_LAUNCHXL.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array. Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* Input pins */
    GPIOCC26XX_DIO_13 | GPIO_DO_NOT_CONFIG,  /* Button 0 */
    GPIOCC26XX_DIO_14 | GPIO_DO_NOT_CONFIG,  /* Button 1 */

    /* Output pins */
    GPIOCC26XX_DIO_07 | GPIO_DO_NOT_CONFIG,  /* Green LED */
    GPIOCC26XX_DIO_06 | GPIO_DO_NOT_CONFIG,  /* Red LED */

    /* SPI Flash CSN */
    GPIOCC26XX_DIO_20 | GPIO_DO_NOT_CONFIG,

    /* SD CS */
    GPIOCC26XX_DIO_21 | GPIO_DO_NOT_CONFIG
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in CC1310_LAUNCH.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
    NULL,  /* Button 0 */
    NULL,  /* Button 1 */
};

const GPIOCC26XX_Config GPIOCC26XX_config = {
    .pinConfigs = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = CC1310_LAUNCHXL_GPIOCOUNT,
    .numberOfCallbacks  = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority = (~0)
};

/*
 *  =============================== GPTimer ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/timer/GPTimerCC26XX.h>

GPTimerCC26XX_Object gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMERCOUNT];

const GPTimerCC26XX_HWAttrs gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMERPARTSCOUNT] = {
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0A, },
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0B, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1A, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1B, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2A, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2B, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3A, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3B, },
};

const GPTimerCC26XX_Config GPTimerCC26XX_config[CC1310_LAUNCHXL_GPTIMERPARTSCOUNT] = {
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER0], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER0A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER0], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER0B], GPT_B },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER1], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER1A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER1], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER1B], GPT_B },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER2], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER2A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER2], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER2B], GPT_B },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER3], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER3A], GPT_A },
    { &gptimerCC26XXObjects[CC1310_LAUNCHXL_GPTIMER3], &gptimerCC26xxHWAttrs[CC1310_LAUNCHXL_GPTIMER3B], GPT_B },
};

/*
 *  =============================== I2C ===============================
*/
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

I2CCC26XX_Object i2cCC26xxObjects[CC1310_LAUNCHXL_I2CCOUNT];

const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CC1310_LAUNCHXL_I2CCOUNT] = {
    {
        .baseAddr    = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum      = INT_I2C_IRQ,
        .intPriority = ~0,
        .swiPriority = 0,
        .sdaPin      = CC1310_LAUNCHXL_I2C0_SDA0,
        .sclPin      = CC1310_LAUNCHXL_I2C0_SCL0,
    }
};

const I2C_Config I2C_config[CC1310_LAUNCHXL_I2CCOUNT] = {
    {
        .fxnTablePtr = &I2CCC26XX_fxnTable,
        .object      = &i2cCC26xxObjects[CC1310_LAUNCHXL_I2C0],
        .hwAttrs     = &i2cCC26xxHWAttrs[CC1310_LAUNCHXL_I2C0]
    }
};

const uint_least8_t I2C_count = CC1310_LAUNCHXL_I2CCOUNT;

/*
 *  =============================== NVS ===============================
 */
#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSSPI25X.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

#define NVS_REGIONS_BASE 0x1B000
#define SECTORSIZE       0x1000
#define REGIONSIZE       (SECTORSIZE * 4)
#define VERIFYBUFSIZE    64

static uint8_t verifyBuf[VERIFYBUFSIZE];

/*
 * Reserve flash sectors for NVS driver use by placing an uninitialized byte
 * array at the desired flash address.
 */
#if defined(__TI_COMPILER_VERSION__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
#pragma LOCATION(flashBuf, NVS_REGIONS_BASE);
#pragma NOINIT(flashBuf);
static char flashBuf[REGIONSIZE];

#elif defined(__IAR_SYSTEMS_ICC__)

/*
 * Place uninitialized array at NVS_REGIONS_BASE
 */
__no_init static char flashBuf[REGIONSIZE] @ NVS_REGIONS_BASE;

#elif defined(__GNUC__)

/*
 * Place the flash buffers in the .nvs section created in the gcc linker file.
 * The .nvs section enforces alignment on a sector boundary but may
 * be placed anywhere in flash memory.  If desired the .nvs section can be set
 * to a fixed address by changing the following in the gcc linker file:
 *
 * .nvs (FIXED_FLASH_ADDR) (NOLOAD) : AT (FIXED_FLASH_ADDR) {
 *      *(.nvs)
 * } > REGION_TEXT
 */
__attribute__ ((section (".nvs")))
static char flashBuf[REGIONSIZE];

#endif

/* Allocate objects for NVS and NVS SPI */
NVSCC26XX_Object nvsCC26xxObjects[1];
NVSSPI25X_Object nvsSPI25XObjects[1];

/* Hardware attributes for NVS */
const NVSCC26XX_HWAttrs nvsCC26xxHWAttrs[1] = {
    {
        .regionBase = (void *)flashBuf,
        .regionSize = REGIONSIZE,
    },
};

/* Hardware attributes for NVS SPI */
const NVSSPI25X_HWAttrs nvsSPI25XHWAttrs[1] = {
    {
        .regionBaseOffset = 0,
        .regionSize = REGIONSIZE,
        .sectorSize = SECTORSIZE,
        .verifyBuf = verifyBuf,
        .verifyBufSize = VERIFYBUFSIZE,
        .spiHandle = NULL,
        .spiIndex = 0,
        .spiBitRate = 4000000,
        .spiCsnGpioIndex = CC1310_LAUNCHXL_GPIO_SPI_FLASH_CS,
    },
};

/* NVS Region index 0 and 1 refer to NVS and NVS SPI respectively */
const NVS_Config NVS_config[CC1310_LAUNCHXL_NVSCOUNT] = {
    {
        .fxnTablePtr = &NVSCC26XX_fxnTable,
        .object = &nvsCC26xxObjects[0],
        .hwAttrs = &nvsCC26xxHWAttrs[0],
    },
    {
        .fxnTablePtr = &NVSSPI25X_fxnTable,
        .object = &nvsSPI25XObjects[0],
        .hwAttrs = &nvsSPI25XHWAttrs[0],
    },
};

const uint_least8_t NVS_count = CC1310_LAUNCHXL_NVSCOUNT;

/*
 *  =============================== PIN ===============================
 */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

const PIN_Config BoardGpioInitTable[] = {

    CC1310_LAUNCHXL_PIN_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,       /* LED initially off          */
    CC1310_LAUNCHXL_PIN_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,       /* LED initially off          */
    CC1310_LAUNCHXL_PIN_BTN1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,          /* Button is active low       */
    CC1310_LAUNCHXL_PIN_BTN2 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,          /* Button is active low       */
    CC1310_LAUNCHXL_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,  /* External flash chip select */
    CC1310_LAUNCHXL_UART_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                              /* UART RX via debugger back channel */
    CC1310_LAUNCHXL_UART_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,                        /* UART TX via debugger back channel */
    CC1310_LAUNCHXL_SPI0_MOSI | PIN_INPUT_EN | PIN_PULLDOWN,                                            /* SPI master out - slave in */
    CC1310_LAUNCHXL_SPI0_MISO | PIN_INPUT_EN | PIN_PULLDOWN,                                            /* SPI master in - slave out */
    CC1310_LAUNCHXL_SPI0_CLK | PIN_INPUT_EN | PIN_PULLDOWN,                                             /* SPI clock */

    PIN_TERMINATE
};

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = ~0,
    .swiPriority = 0
};

/*
 *  =============================== Power ===============================
 */
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

const PowerCC26XX_Config PowerCC26XX_config = {
    .policyInitFxn      = NULL,
    .policyFxn          = &PowerCC26XX_standbyPolicy,
    .calibrateFxn       = &PowerCC26XX_calibrate,
    .enablePolicy       = true,
    .calibrateRCOSC_LF  = true,
    .calibrateRCOSC_HF  = true,
};

/*
 *  =============================== PWM ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTimerCC26XX.h>

PWMTimerCC26XX_Object pwmtimerCC26xxObjects[CC1310_LAUNCHXL_PWMCOUNT];

const PWMTimerCC26XX_HwAttrs pwmtimerCC26xxHWAttrs[CC1310_LAUNCHXL_PWMCOUNT] = {
    { .pwmPin = CC1310_LAUNCHXL_PWMPIN0, .gpTimerUnit = CC1310_LAUNCHXL_GPTIMER0A },
    { .pwmPin = CC1310_LAUNCHXL_PWMPIN1, .gpTimerUnit = CC1310_LAUNCHXL_GPTIMER0B },
    { .pwmPin = CC1310_LAUNCHXL_PWMPIN2, .gpTimerUnit = CC1310_LAUNCHXL_GPTIMER1A },
    { .pwmPin = CC1310_LAUNCHXL_PWMPIN3, .gpTimerUnit = CC1310_LAUNCHXL_GPTIMER1B },
    { .pwmPin = CC1310_LAUNCHXL_PWMPIN4, .gpTimerUnit = CC1310_LAUNCHXL_GPTIMER2A },
    { .pwmPin = CC1310_LAUNCHXL_PWMPIN5, .gpTimerUnit = CC1310_LAUNCHXL_GPTIMER2B },
    { .pwmPin = CC1310_LAUNCHXL_PWMPIN6, .gpTimerUnit = CC1310_LAUNCHXL_GPTIMER3A },
    { .pwmPin = CC1310_LAUNCHXL_PWMPIN7, .gpTimerUnit = CC1310_LAUNCHXL_GPTIMER3B },
};

const PWM_Config PWM_config[CC1310_LAUNCHXL_PWMCOUNT] = {
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1310_LAUNCHXL_PWM0], &pwmtimerCC26xxHWAttrs[CC1310_LAUNCHXL_PWM0] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1310_LAUNCHXL_PWM1], &pwmtimerCC26xxHWAttrs[CC1310_LAUNCHXL_PWM1] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1310_LAUNCHXL_PWM2], &pwmtimerCC26xxHWAttrs[CC1310_LAUNCHXL_PWM2] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1310_LAUNCHXL_PWM3], &pwmtimerCC26xxHWAttrs[CC1310_LAUNCHXL_PWM3] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1310_LAUNCHXL_PWM4], &pwmtimerCC26xxHWAttrs[CC1310_LAUNCHXL_PWM4] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1310_LAUNCHXL_PWM5], &pwmtimerCC26xxHWAttrs[CC1310_LAUNCHXL_PWM5] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1310_LAUNCHXL_PWM6], &pwmtimerCC26xxHWAttrs[CC1310_LAUNCHXL_PWM6] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[CC1310_LAUNCHXL_PWM7], &pwmtimerCC26xxHWAttrs[CC1310_LAUNCHXL_PWM7] },
};

const uint_least8_t PWM_count = CC1310_LAUNCHXL_PWMCOUNT;

/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

const RFCC26XX_HWAttrs RFCC26XX_hwAttrs = {
    .hwiCpe0Priority = ~0,
    .hwiHwPriority   = ~0,
    .swiCpe0Priority =  0,
    .swiHwPriority   =  0,
};

/*
 *  =============================== SD ===============================
 */
#if 0
#include <ti/drivers/SD.h>
#include <ti/drivers/sd/SDSPI.h>

SDSPI_Object sdspiObjects[CC1310_LAUNCHXL_SDCOUNT];

const SDSPI_HWAttrs sdspiHWAttrs[CC1310_LAUNCHXL_SDCOUNT] = {
    {
        .spiIndex = CC1310_LAUNCHXL_SPI0,
        .spiCsGpioIndex = CC1310_LAUNCHXL_SDSPI_CS
    }
};

const SD_Config SD_config[CC1310_LAUNCHXL_SDCOUNT] = {
    {
        .fxnTablePtr = &SDSPI_fxnTable,
        .object = &sdspiObjects[CC1310_LAUNCHXL_SDSPI0],
        .hwAttrs = &sdspiHWAttrs[CC1310_LAUNCHXL_SDSPI0]
    },
};

const uint_least8_t SD_count = CC1310_LAUNCHXL_SDCOUNT;
#endif

/*
 *  =============================== SPI DMA ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

SPICC26XXDMA_Object spiCC26XXDMAObjects[CC1310_LAUNCHXL_SPICOUNT];

/*
 * NOTE: The SPI instances below can be used by the SD driver to communicate
 * with a SD card via SPI.  The 'defaultTxBufValue' fields below are set to 0xFF
 * to satisfy the SDSPI driver requirement.
 */
const SPICC26XXDMA_HWAttrsV1 spiCC26XXDMAHWAttrs[CC1310_LAUNCHXL_SPICOUNT] = {
    {
        .baseAddr           = SSI0_BASE,
        .intNum             = INT_SSI0_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI0,
        .defaultTxBufValue  = 0xFF,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI0_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI0_TX,
        .mosiPin            = CC1310_LAUNCHXL_SPI0_MOSI,
        .misoPin            = CC1310_LAUNCHXL_SPI0_MISO,
        .clkPin             = CC1310_LAUNCHXL_SPI0_CLK,
        .csnPin             = CC1310_LAUNCHXL_SPI0_CSN,
//        .minDmaTransferSize = 10
    },
    {
        .baseAddr           = SSI1_BASE,
        .intNum             = INT_SSI1_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI1,
        .defaultTxBufValue  = 0xFF,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI1_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI1_TX,
        .mosiPin            = CC1310_LAUNCHXL_SPI1_MOSI,
        .misoPin            = CC1310_LAUNCHXL_SPI1_MISO,
        .clkPin             = CC1310_LAUNCHXL_SPI1_CLK,
        .csnPin             = CC1310_LAUNCHXL_SPI1_CSN,
//        .minDmaTransferSize = 10
    }
};

const SPI_Config SPI_config[CC1310_LAUNCHXL_SPICOUNT] = {
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[CC1310_LAUNCHXL_SPI0],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[CC1310_LAUNCHXL_SPI0]
    },
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[CC1310_LAUNCHXL_SPI1],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[CC1310_LAUNCHXL_SPI1]
    },
};

const uint_least8_t SPI_count = CC1310_LAUNCHXL_SPICOUNT;

/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

UARTCC26XX_Object uartCC26XXObjects[CC1310_LAUNCHXL_UARTCOUNT];

uint8_t uartCC26XXRingBuffer[CC1310_LAUNCHXL_UARTCOUNT][32];

const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs[CC1310_LAUNCHXL_UARTCOUNT] = {
    {
        .baseAddr       = UART0_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_UART0,
        .intNum         = INT_UART0_COMB,
        .intPriority    = ~0,
        .swiPriority    = 0,
        .txPin          = CC1310_LAUNCHXL_UART_TX,
        .rxPin          = CC1310_LAUNCHXL_UART_RX,
        .ctsPin         = PIN_UNASSIGNED,
        .rtsPin         = PIN_UNASSIGNED,
        .ringBufPtr     = uartCC26XXRingBuffer[CC1310_LAUNCHXL_UART0],
        .ringBufSize    = sizeof(uartCC26XXRingBuffer[CC1310_LAUNCHXL_UART0]),
//        .txIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_1_8,
//        .rxIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_4_8
    }
};

const UART_Config UART_config[CC1310_LAUNCHXL_UARTCOUNT] = {
    {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects[CC1310_LAUNCHXL_UART0],
        .hwAttrs     = &uartCC26XXHWAttrs[CC1310_LAUNCHXL_UART0]
    },
};

const uint_least8_t UART_count = CC1310_LAUNCHXL_UARTCOUNT;

/*
 *  =============================== UDMA ===============================
 */
#include <ti/drivers/dma/UDMACC26XX.h>

UDMACC26XX_Object udmaObjects[CC1310_LAUNCHXL_UDMACOUNT];

const UDMACC26XX_HWAttrs udmaHWAttrs[CC1310_LAUNCHXL_UDMACOUNT] = {
    {
        .baseAddr    = UDMA0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_UDMA,
        .intNum      = INT_DMA_ERR,
        .intPriority = ~0
    }
};

const UDMACC26XX_Config UDMACC26XX_config[CC1310_LAUNCHXL_UDMACOUNT] = {
    {
         .object  = &udmaObjects[CC1310_LAUNCHXL_UDMA0],
         .hwAttrs = &udmaHWAttrs[CC1310_LAUNCHXL_UDMA0]
    },
};

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>

WatchdogCC26XX_Object watchdogCC26XXObjects[CC1310_LAUNCHXL_WATCHDOGCOUNT];

const WatchdogCC26XX_HWAttrs watchdogCC26XXHWAttrs[CC1310_LAUNCHXL_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WDT_BASE,
        .reloadValue = 1000 /* Reload value in milliseconds */
    },
};

const Watchdog_Config Watchdog_config[CC1310_LAUNCHXL_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogCC26XX_fxnTable,
        .object      = &watchdogCC26XXObjects[CC1310_LAUNCHXL_WATCHDOG0],
        .hwAttrs     = &watchdogCC26XXHWAttrs[CC1310_LAUNCHXL_WATCHDOG0]
    },
};

const uint_least8_t Watchdog_count = CC1310_LAUNCHXL_WATCHDOGCOUNT;

/*
 *  ======== CC1310_LAUNCHXL_wakeUpExtFlash ========
 */
void CC1310_LAUNCHXL_wakeUpExtFlash(void)
{
    PIN_Config extFlashPinTable[] = {
        CC1310_LAUNCHXL_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_INPUT_DIS | PIN_DRVSTR_MED,
        PIN_TERMINATE
    };
    PIN_State extFlashPinState;
    PIN_Handle extFlashPinHandle = PIN_open(&extFlashPinState, extFlashPinTable);

    /*
     *  To wake up we need to toggle the chip select at
     *  least 20 ns and ten wait at least 35 us.
     */

    /* Toggle chip select for ~20ns to wake ext. flash */
    PIN_setOutputValue(extFlashPinHandle, CC1310_LAUNCHXL_SPI_FLASH_CS, 0);
    /* 3 cycles per loop: 1 loop @ 48 Mhz ~= 62 ns */
    CPUdelay(1);
    PIN_setOutputValue(extFlashPinHandle, CC1310_LAUNCHXL_SPI_FLASH_CS, 1);
    /* 3 cycles per loop: 560 loops @ 48 Mhz ~= 35 us */
    CPUdelay(560);

    PIN_close(extFlashPinHandle);
}

/*
 *  ======== CC1310_LAUNCHXL_sendExtFlashByte ========
 */
void CC1310_LAUNCHXL_sendExtFlashByte(PIN_Handle pinHandle, uint8_t byte)
{
    uint8_t i;

    PIN_setOutputValue(pinHandle, CC1310_LAUNCHXL_SPI_FLASH_CS, 0);

    for (i = 0; i < 8; i++) {
        PIN_setOutputValue(pinHandle, CC1310_LAUNCHXL_SPI0_CLK, 0);
        PIN_setOutputValue(pinHandle, CC1310_LAUNCHXL_SPI0_MOSI, (byte >> (7 - i)) & 0x01);
        PIN_setOutputValue(pinHandle, CC1310_LAUNCHXL_SPI0_CLK, 1);

        /*
         * Waste a few cycles to keep the CLK high for at
         * least 45% of the period.
         * 3 cycles per loop: 8 loops @ 48 Mhz = 0.5 us.
         */
        CPUdelay(8);
    }

    PIN_setOutputValue(pinHandle, CC1310_LAUNCHXL_SPI0_CLK, 0);
    PIN_setOutputValue(pinHandle, CC1310_LAUNCHXL_SPI_FLASH_CS, 1);

    /*
     * Keep CS high at least 40 us
     * 3 cycles per loop: 700 loops @ 48 Mhz ~= 44 us
     */
    CPUdelay(700);
}

/*
 *  ======== CC1310_LAUNCHXL_shutDownExtFlash ========
 */
void CC1310_LAUNCHXL_shutDownExtFlash(void)
{
    /* To be sure we are putting the flash into sleep and not waking it, we first have to make a wake up call */
    CC1310_LAUNCHXL_wakeUpExtFlash();

    PIN_Config extFlashPinTable[] = {
        CC1310_LAUNCHXL_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_INPUT_DIS | PIN_DRVSTR_MED,
        CC1310_LAUNCHXL_SPI0_CLK | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_INPUT_DIS | PIN_DRVSTR_MED,
        CC1310_LAUNCHXL_SPI0_MOSI | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_INPUT_DIS | PIN_DRVSTR_MED,
        CC1310_LAUNCHXL_SPI0_MISO | PIN_INPUT_EN | PIN_PULLDOWN,
        PIN_TERMINATE
    };
    PIN_State extFlashPinState;
    PIN_Handle extFlashPinHandle = PIN_open(&extFlashPinState, extFlashPinTable);

    uint8_t extFlashShutdown = 0xB9;

    CC1310_LAUNCHXL_sendExtFlashByte(extFlashPinHandle, extFlashShutdown);

    PIN_close(extFlashPinHandle);
}

/*
 *  ======== CC1310_LAUNCHXL_initGeneral ========
 */
void CC1310_LAUNCHXL_initGeneral(void)
{
    Power_init();

    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) {
        /* Error with PIN_init */
        while (1);
    }

    /* Shut down external flash as default */
    CC1310_LAUNCHXL_shutDownExtFlash();
}
