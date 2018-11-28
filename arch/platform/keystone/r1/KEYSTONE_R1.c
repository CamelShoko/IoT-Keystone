/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
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
 *  ====================== KEYSTONE_R1.c ===================================
 *  This file is responsible for setting up the board specific items for the
 *  KEYSTONE_R1 board.
 */

#include "contiki.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)
#include DeviceFamily_constructPath(driverlib/udma.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)

#include "KEYSTONE_R1.h"

/*
 *  =============================== ADCBuf ===============================
 */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/adcbuf/ADCBufCC26X2.h>

ADCBufCC26X2_Object adcBufCC26xxObjects[KEYSTONE_R1_ADCBUFCOUNT];

/*
 *  This table converts a virtual adc channel into a dio and internal analogue
 *  input signal. This table is necessary for the functioning of the adcBuf
 *  driver. Comment out unused entries to save flash. Dio and internal signal
 *  pairs are hardwired. Do not remap them in the table. You may reorder entire
 *  entries. The mapping of dio and internal signals is package dependent.
 */
const ADCBufCC26X2_AdcChannelLutEntry ADCBufCC26X2_adcChannelLut[KEYSTONE_R1_ADCBUF0CHANNELCOUNT] = {
    {KEYSTONE_R1_DIO23_ANALOG, ADC_COMPB_IN_AUXIO7},
    {KEYSTONE_R1_DIO24_ANALOG, ADC_COMPB_IN_AUXIO6},
    {KEYSTONE_R1_DIO25_ANALOG, ADC_COMPB_IN_AUXIO5},
    {KEYSTONE_R1_DIO26_ANALOG, ADC_COMPB_IN_AUXIO4},
    {KEYSTONE_R1_DIO27_ANALOG, ADC_COMPB_IN_AUXIO3},
    {KEYSTONE_R1_DIO28_ANALOG, ADC_COMPB_IN_AUXIO2},
    {KEYSTONE_R1_DIO29_ANALOG, ADC_COMPB_IN_AUXIO1},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VDDS},
    {PIN_UNASSIGNED, ADC_COMPB_IN_DCOUPL},
    {PIN_UNASSIGNED, ADC_COMPB_IN_VSS},
};

const ADCBufCC26X2_HWAttrs adcBufCC26xxHWAttrs[KEYSTONE_R1_ADCBUFCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
        .adcChannelLut     = ADCBufCC26X2_adcChannelLut,
        .gpTimerUnit       = KEYSTONE_R1_GPTIMER0A,
    }
};

const ADCBuf_Config ADCBuf_config[KEYSTONE_R1_ADCBUFCOUNT] = {
    {
        &ADCBufCC26X2_fxnTable,
        &adcBufCC26xxObjects[KEYSTONE_R1_ADCBUF0],
        &adcBufCC26xxHWAttrs[KEYSTONE_R1_ADCBUF0]
    },
};

const uint_least8_t ADCBuf_count = KEYSTONE_R1_ADCBUFCOUNT;

/*
 *  =============================== ADC ===============================
 */
#include <ti/drivers/ADC.h>
#include <ti/drivers/adc/ADCCC26XX.h>

ADCCC26XX_Object adcCC26xxObjects[KEYSTONE_R1_ADCCOUNT];

const ADCCC26XX_HWAttrs adcCC26xxHWAttrs[KEYSTONE_R1_ADCCOUNT] = {
    {
        .adcDIO              = KEYSTONE_R1_DIO23_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO7,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = KEYSTONE_R1_DIO24_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO6,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = KEYSTONE_R1_DIO25_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO5,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = KEYSTONE_R1_DIO26_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO4,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = KEYSTONE_R1_DIO27_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO3,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = KEYSTONE_R1_DIO28_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO2,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
        .inputScalingEnabled = true,
        .triggerSource       = ADCCC26XX_TRIGGER_MANUAL,
        .returnAdjustedVal   = false
    },
    {
        .adcDIO              = KEYSTONE_R1_DIO29_ANALOG,
        .adcCompBInput       = ADC_COMPB_IN_AUXIO1,
        .refSource           = ADCCC26XX_FIXED_REFERENCE,
        .samplingDuration    = ADCCC26XX_SAMPLING_DURATION_2P7_US,
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

const ADC_Config ADC_config[KEYSTONE_R1_ADCCOUNT] = {
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADC0], &adcCC26xxHWAttrs[KEYSTONE_R1_ADC0]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADC1], &adcCC26xxHWAttrs[KEYSTONE_R1_ADC1]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADC2], &adcCC26xxHWAttrs[KEYSTONE_R1_ADC2]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADC3], &adcCC26xxHWAttrs[KEYSTONE_R1_ADC3]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADC4], &adcCC26xxHWAttrs[KEYSTONE_R1_ADC4]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADC5], &adcCC26xxHWAttrs[KEYSTONE_R1_ADC5]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADC6], &adcCC26xxHWAttrs[KEYSTONE_R1_ADC6]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADCDCOUPL], &adcCC26xxHWAttrs[KEYSTONE_R1_ADCDCOUPL]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADCVSS], &adcCC26xxHWAttrs[KEYSTONE_R1_ADCVSS]},
    {&ADCCC26XX_fxnTable, &adcCC26xxObjects[KEYSTONE_R1_ADCVDDS], &adcCC26xxHWAttrs[KEYSTONE_R1_ADCVDDS]},
};

const uint_least8_t ADC_count = KEYSTONE_R1_ADCCOUNT;

/*
 *  =============================== ECDH ===============================
 */
#include <ti/drivers/ECDH.h>
#include <ti/drivers/ecdh/ECDHCC26X2.h>

ECDHCC26X2_Object ecdhCC26X2Objects[KEYSTONE_R1_ECDHCOUNT];

const ECDHCC26X2_HWAttrs ecdhCC26X2HWAttrs[KEYSTONE_R1_ECDHCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
    }
};

const ECDH_Config ECDH_config[KEYSTONE_R1_ECDHCOUNT] = {
    {
         .object  = &ecdhCC26X2Objects[KEYSTONE_R1_ECDH0],
         .hwAttrs = &ecdhCC26X2HWAttrs[KEYSTONE_R1_ECDH0]
    },
};

const uint_least8_t ECDH_count = KEYSTONE_R1_ECDHCOUNT;

/*
 *  =============================== ECDSA ===============================
 */
#include <ti/drivers/ECDSA.h>
#include <ti/drivers/ecdsa/ECDSACC26X2.h>

ECDSACC26X2_Object ecdsaCC26X2Objects[KEYSTONE_R1_ECDSACOUNT];

const ECDSACC26X2_HWAttrs ecdsaCC26X2HWAttrs[KEYSTONE_R1_ECDSACOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
    }
};

const ECDSA_Config ECDSA_config[KEYSTONE_R1_ECDSACOUNT] = {
    {
         .object  = &ecdsaCC26X2Objects[KEYSTONE_R1_ECDSA0],
         .hwAttrs = &ecdsaCC26X2HWAttrs[KEYSTONE_R1_ECDSA0]
    },
};

const uint_least8_t ECDSA_count = KEYSTONE_R1_ECDSACOUNT;

/*
 *  =============================== ECJPAKE ===============================
 */
#include <ti/drivers/ECJPAKE.h>
#include <ti/drivers/ecjpake/ECJPAKECC26X2.h>

ECJPAKECC26X2_Object ecjpakeCC26X2Objects[KEYSTONE_R1_ECJPAKECOUNT];

const ECJPAKECC26X2_HWAttrs ecjpakeCC26X2HWAttrs[KEYSTONE_R1_ECJPAKECOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
    }
};

const ECJPAKE_Config ECJPAKE_config[KEYSTONE_R1_ECJPAKECOUNT] = {
    {
         .object  = &ecjpakeCC26X2Objects[KEYSTONE_R1_ECJPAKE0],
         .hwAttrs = &ecjpakeCC26X2HWAttrs[KEYSTONE_R1_ECJPAKE0]
    },
};

const uint_least8_t ECJPAKE_count = KEYSTONE_R1_ECJPAKECOUNT;


/*
 *  =============================== SHA2 ===============================
 */
#include <ti/drivers/SHA2.h>
#include <ti/drivers/sha2/SHA2CC26X2.h>

SHA2CC26X2_Object sha2CC26X2Objects[KEYSTONE_R1_SHA2COUNT];

const SHA2CC26X2_HWAttrs sha2CC26X2HWAttrs[KEYSTONE_R1_SHA2COUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
    }
};

const SHA2_Config SHA2_config[KEYSTONE_R1_SHA2COUNT] = {
    {
         .object  = &sha2CC26X2Objects[KEYSTONE_R1_SHA20],
         .hwAttrs = &sha2CC26X2HWAttrs[KEYSTONE_R1_SHA20]
    },
};

const uint_least8_t SHA2_count = KEYSTONE_R1_SHA2COUNT;

/*
 *  =============================== AESCCM ===============================
 */
#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMCC26XX.h>

AESCCMCC26XX_Object aesccmCC26XXObjects[KEYSTONE_R1_AESCCMCOUNT];

const AESCCMCC26XX_HWAttrs aesccmCC26XXHWAttrs[KEYSTONE_R1_AESCCMCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
    }
};

const AESCCM_Config AESCCM_config[KEYSTONE_R1_AESCCMCOUNT] = {
    {
         .object  = &aesccmCC26XXObjects[KEYSTONE_R1_AESCCM0],
         .hwAttrs = &aesccmCC26XXHWAttrs[KEYSTONE_R1_AESCCM0]
    },
};

const uint_least8_t AESCCM_count = KEYSTONE_R1_AESCCMCOUNT;

/*
 *  =============================== AESECB ===============================
 */
#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBCC26XX.h>

AESECBCC26XX_Object aesecbCC26XXObjects[KEYSTONE_R1_AESECBCOUNT];

const AESECBCC26XX_HWAttrs aesecbCC26XXHWAttrs[KEYSTONE_R1_AESECBCOUNT] = {
    {
        .intPriority       = ~0,
        .swiPriority       = 0,
    }
};

const AESECB_Config AESECB_config[KEYSTONE_R1_AESECBCOUNT] = {
    {
         .object  = &aesecbCC26XXObjects[KEYSTONE_R1_AESECB0],
         .hwAttrs = &aesecbCC26XXHWAttrs[KEYSTONE_R1_AESECB0]
    },
};

const uint_least8_t AESECB_count = KEYSTONE_R1_AESECBCOUNT;

/*
 *  =============================== GPTimer ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/timer/GPTimerCC26XX.h>

GPTimerCC26XX_Object gptimerCC26XXObjects[KEYSTONE_R1_GPTIMERCOUNT];

const GPTimerCC26XX_HWAttrs gptimerCC26xxHWAttrs[KEYSTONE_R1_GPTIMERPARTSCOUNT] = {
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0A, },
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0B, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1A, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1B, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2A, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2B, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3A, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3B, },
};

const GPTimerCC26XX_Config GPTimerCC26XX_config[KEYSTONE_R1_GPTIMERPARTSCOUNT] = {
    { &gptimerCC26XXObjects[KEYSTONE_R1_GPTIMER0], &gptimerCC26xxHWAttrs[KEYSTONE_R1_GPTIMER0A], GPT_A },
    { &gptimerCC26XXObjects[KEYSTONE_R1_GPTIMER0], &gptimerCC26xxHWAttrs[KEYSTONE_R1_GPTIMER0B], GPT_B },
    { &gptimerCC26XXObjects[KEYSTONE_R1_GPTIMER1], &gptimerCC26xxHWAttrs[KEYSTONE_R1_GPTIMER1A], GPT_A },
    { &gptimerCC26XXObjects[KEYSTONE_R1_GPTIMER1], &gptimerCC26xxHWAttrs[KEYSTONE_R1_GPTIMER1B], GPT_B },
    { &gptimerCC26XXObjects[KEYSTONE_R1_GPTIMER2], &gptimerCC26xxHWAttrs[KEYSTONE_R1_GPTIMER2A], GPT_A },
    { &gptimerCC26XXObjects[KEYSTONE_R1_GPTIMER2], &gptimerCC26xxHWAttrs[KEYSTONE_R1_GPTIMER2B], GPT_B },
    { &gptimerCC26XXObjects[KEYSTONE_R1_GPTIMER3], &gptimerCC26xxHWAttrs[KEYSTONE_R1_GPTIMER3A], GPT_A },
    { &gptimerCC26XXObjects[KEYSTONE_R1_GPTIMER3], &gptimerCC26xxHWAttrs[KEYSTONE_R1_GPTIMER3B], GPT_B },
};

/*
 *  =============================== I2C ===============================
 */
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

#if TI_I2C_CONF_ENABLE

I2CCC26XX_Object i2cCC26xxObjects[KEYSTONE_R1_I2CCOUNT];

const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[KEYSTONE_R1_I2CCOUNT] = {
#if TI_I2C_CONF_I2C0_ENABLE
    {
        .baseAddr    = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum      = INT_I2C_IRQ,
        .intPriority = ~0,
        .swiPriority = 0,
        .sdaPin      = KEYSTONE_R1_I2C0_SDA0,
        .sclPin      = KEYSTONE_R1_I2C0_SCL0,
    },
#endif
};

const I2C_Config I2C_config[KEYSTONE_R1_I2CCOUNT] = {
#if TI_I2C_CONF_I2C0_ENABLE
    {
        .fxnTablePtr = &I2CCC26XX_fxnTable,
        .object      = &i2cCC26xxObjects[KEYSTONE_R1_I2C0],
        .hwAttrs     = &i2cCC26xxHWAttrs[KEYSTONE_R1_I2C0]
    },
#endif
};

const uint_least8_t I2C_count = KEYSTONE_R1_I2CCOUNT;

#endif /* TI_I2C_CONF_ENABLE */

/*
 *  =============================== NVS ===============================
 */
#include <ti/drivers/NVS.h>
#include <ti/drivers/nvs/NVSSPI25X.h>
#include <ti/drivers/nvs/NVSCC26XX.h>

#define NVS_REGIONS_BASE 0x48000
#define SECTORSIZE       0x2000
#define REGIONSIZE       (SECTORSIZE * 4)

#if TI_NVS_CONF_ENABLE

#if TI_NVS_CONF_NVS_INTERNAL_ENABLE

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
static __no_init char flashBuf[REGIONSIZE] @ NVS_REGIONS_BASE;

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

/* Allocate objects for NVS Internal Regions */
NVSCC26XX_Object nvsCC26xxObjects[1];

/* Hardware attributes for NVS Internal Regions */
const NVSCC26XX_HWAttrs nvsCC26xxHWAttrs[1] = {
    {
        .regionBase = (void *)flashBuf,
        .regionSize = REGIONSIZE,
    },
};

#endif /* TI_NVS_CONF_NVS_INTERNAL_ENABLE */

#if TI_NVS_CONF_NVS_EXTERNAL_ENABLE

#define SPISECTORSIZE    0x1000
#define SPIREGIONSIZE    (SPISECTORSIZE * 32)
#define VERIFYBUFSIZE    64

static uint8_t verifyBuf[VERIFYBUFSIZE];

/* Allocate objects for NVS External Regions */
NVSSPI25X_Object nvsSPI25XObjects[1];

/* Hardware attributes for NVS External Regions */
const NVSSPI25X_HWAttrs nvsSPI25XHWAttrs[1] = {
    {
        .regionBaseOffset = 0,
        .regionSize = SPIREGIONSIZE,
        .sectorSize = SPISECTORSIZE,
        .verifyBuf = verifyBuf,
        .verifyBufSize = VERIFYBUFSIZE,
        .spiHandle = NULL,
        .spiIndex = 0,
        .spiBitRate = 4000000,
        .spiCsnGpioIndex = KEYSTONE_R1_GPIO_SPI_FLASH_CS,
    },
};

#endif /* TI_NVS_CONF_NVS_EXTERNAL_ENABLE */

/* NVS Region index 0 and 1 refer to NVS and NVS SPI respectively */
const NVS_Config NVS_config[KEYSTONE_R1_NVSCOUNT] = {
#if TI_NVS_CONF_NVS_INTERNAL_ENABLE
    {
        .fxnTablePtr = &NVSCC26XX_fxnTable,
        .object = &nvsCC26xxObjects[0],
        .hwAttrs = &nvsCC26xxHWAttrs[0],
    },
#endif
#if TI_NVS_CONF_NVS_EXTERNAL_ENABLE
    {
        .fxnTablePtr = &NVSSPI25X_fxnTable,
        .object = &nvsSPI25XObjects[0],
        .hwAttrs = &nvsSPI25XHWAttrs[0],
    },
#endif
};

const uint_least8_t NVS_count = KEYSTONE_R1_NVSCOUNT;

#endif /* TI_NVS_CONF_ENABLE */

/*
 *  =============================== PIN ===============================
 */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

/*
 * When configuring IO, note the following:
 *   - IO connected as GPIO output should have PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW/HIGH  
 *   - IO connected to internal peripherals like SPI should only have the input buffer
 *     enabled with PIN_INPUT_EN
 */

/*
 * This is the INITIAL state of IO pins on boot up as applied by PIN_init().  The pin states
 * and configuration CAN CHANGE at a later time by the device drivers that use them when
 * they call PIN_open() on the specific pins they control.
 * 
 * It is also possible that drivers will access the pins directly without access control
 * using the PINCC26XX_setOutputValue() API.
*/

const PIN_Config BoardGpioInitTable[] = {

#if 0 /* LED initialized through GPIO hal. Leave them tri-state at startup (off). */
    /* LED */
    KEYSTONE_R1_PIN_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,          /* LED initially off */
    KEYSTONE_R1_PIN_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,          /* LED initially off */
#endif

#if 0
    /* Buttons */
    KEYSTONE_R1_PIN_BTN1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,             /* Button is active low */
    KEYSTONE_R1_PIN_BTN2 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES | PIN_HYSTERESIS,             /* Button is active low */
#endif

    /* Chip selects */
    KEYSTONE_R1_SPI_FLASH_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,     /* External flash chip select: deselected */
    KEYSTONE_R1_SPI_LORA_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,      /* LoRa radio chip select: deselected */
    KEYSTONE_R1_SPI_BME280_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,    /* BME280 sensor chip select: deselected */
    KEYSTONE_R1_SPI_IMU_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,       /* IMU chip select: deselected */

    /* GPIO */
    //KEYSTONE_R1_PIN_LORA_INT | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE,                          /* SX1262 INT active high */
    KEYSTONE_R1_PIN_IMU_INT | PIN_INPUT_EN | PIN_PULLDOWN | PIN_IRQ_POSEDGE,                           /* SX1262 INT active high */
    KEYSTONE_R1_PIN_ALS_INT | PIN_INPUT_EN | PIN_IRQ_NEGEDGE,                                          /* OPT3001 light sensor INT open drain active low external 10K pullup*/
    KEYSTONE_R1_PIN_HALL | PIN_INPUT_EN,                                                               /* HALL sensor digital input */
    KEYSTONE_R1_PIN_FLASH_EN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,     /* Flash ENABLE output active low: disabled */


    /* UART IO */
    KEYSTONE_R1_UART0_RX | PIN_INPUT_EN | PIN_PULLDOWN,                                                /* UART RX via debugger back channel */
    KEYSTONE_R1_UART0_TX | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL,                          /* UART TX via debugger back channel */

    /* SPI IO */
    KEYSTONE_R1_SPI0_MOSI | PIN_INPUT_EN | PIN_PULLDOWN,                                               /* SPI0 master out - slave in */
    KEYSTONE_R1_SPI0_MISO | PIN_INPUT_EN | PIN_PULLDOWN,                                               /* SPI0 master in - slave out */
    KEYSTONE_R1_SPI0_CLK | PIN_INPUT_EN | PIN_PULLDOWN,                                                /* SPI0 clock */
    KEYSTONE_R1_SPI1_MOSI | PIN_INPUT_EN | PIN_PULLDOWN,                                               /* SPI1 master out - slave in */
    KEYSTONE_R1_SPI1_MISO | PIN_INPUT_EN | PIN_PULLDOWN,                                               /* SPI1 master in - slave out */
    KEYSTONE_R1_SPI1_CLK | PIN_INPUT_EN | PIN_PULLDOWN,                                                /* SPI1 clock */

    /* I2C IO */
    KEYSTONE_R1_I2C0_SCL0 | PIN_INPUT_EN | PIN_OPENDRAIN,                                              /* I2C bus has external pull-ups */
    KEYSTONE_R1_I2C0_SDA0 | PIN_INPUT_EN | PIN_OPENDRAIN,                                              /* I2C bus has external pull-ups */

    /* RF */
    KEYSTONE_R1_RF_SUB1GHZ | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,        /* RF Switch defaults to CC1352 path */
    KEYSTONE_R1_RF_SUB1GHZ_NCTRL | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,  /* RF Switch on/off */
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
#include <ti/drivers/power/PowerCC26X2.h>

const PowerCC26X2_Config PowerCC26X2_config = {
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

PWMTimerCC26XX_Object pwmtimerCC26xxObjects[KEYSTONE_R1_PWMCOUNT];

const PWMTimerCC26XX_HwAttrs pwmtimerCC26xxHWAttrs[KEYSTONE_R1_PWMCOUNT] = {
    { .pwmPin = KEYSTONE_R1_PWMPIN0, .gpTimerUnit = KEYSTONE_R1_GPTIMER0A },
    { .pwmPin = KEYSTONE_R1_PWMPIN1, .gpTimerUnit = KEYSTONE_R1_GPTIMER0B },
    { .pwmPin = KEYSTONE_R1_PWMPIN2, .gpTimerUnit = KEYSTONE_R1_GPTIMER1A },
    { .pwmPin = KEYSTONE_R1_PWMPIN3, .gpTimerUnit = KEYSTONE_R1_GPTIMER1B },
    { .pwmPin = KEYSTONE_R1_PWMPIN4, .gpTimerUnit = KEYSTONE_R1_GPTIMER2A },
    { .pwmPin = KEYSTONE_R1_PWMPIN5, .gpTimerUnit = KEYSTONE_R1_GPTIMER2B },
    { .pwmPin = KEYSTONE_R1_PWMPIN6, .gpTimerUnit = KEYSTONE_R1_GPTIMER3A },
    { .pwmPin = KEYSTONE_R1_PWMPIN7, .gpTimerUnit = KEYSTONE_R1_GPTIMER3B },
};

const PWM_Config PWM_config[KEYSTONE_R1_PWMCOUNT] = {
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[KEYSTONE_R1_PWM0], &pwmtimerCC26xxHWAttrs[KEYSTONE_R1_PWM0] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[KEYSTONE_R1_PWM1], &pwmtimerCC26xxHWAttrs[KEYSTONE_R1_PWM1] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[KEYSTONE_R1_PWM2], &pwmtimerCC26xxHWAttrs[KEYSTONE_R1_PWM2] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[KEYSTONE_R1_PWM3], &pwmtimerCC26xxHWAttrs[KEYSTONE_R1_PWM3] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[KEYSTONE_R1_PWM4], &pwmtimerCC26xxHWAttrs[KEYSTONE_R1_PWM4] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[KEYSTONE_R1_PWM5], &pwmtimerCC26xxHWAttrs[KEYSTONE_R1_PWM5] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[KEYSTONE_R1_PWM6], &pwmtimerCC26xxHWAttrs[KEYSTONE_R1_PWM6] },
    { &PWMTimerCC26XX_fxnTable, &pwmtimerCC26xxObjects[KEYSTONE_R1_PWM7], &pwmtimerCC26xxHWAttrs[KEYSTONE_R1_PWM7] },
};

const uint_least8_t PWM_count = KEYSTONE_R1_PWMCOUNT;

/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

/*
 *  Board-specific callback function to set the correct antenna path.
 *
 *  This function is called by the RF driver on global driver events.
 *  It contains a default implementation to set the correct antenna path.
 *  This function is defined in the file KEYSTONE_R1_fxns.c
 */
extern void rfDriverCallback(RF_Handle client, RF_GlobalEvent events, void *arg);

const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = ~0,     /* Lowest HWI priority */
    .swiPriority        = 0,      /* Lowest SWI priority */
    .xoscHfAlwaysNeeded = true,   /* Keep XOSC dependency while in stanby */

    /* Register the board specific callback */
    .globalCallback     = &rfDriverCallback,

    /* Subscribe the callback to both events */
    .globalEventMask    = RF_GlobalEventRadioSetup | RF_GlobalEventRadioPowerDown
};


/*
 *  =============================== SPI DMA ===============================
 */
#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>

#if TI_SPI_CONF_ENABLE

SPICC26XXDMA_Object spiCC26XXDMAObjects[KEYSTONE_R1_SPICOUNT];

/*
 * NOTE: The SPI instances below can be used by the SD driver to communicate
 * with a SD card via SPI.  The 'defaultTxBufValue' fields below are set to 0xFF
 * to satisfy the SDSPI driver requirement.
 */
const SPICC26XXDMA_HWAttrsV1 spiCC26XXDMAHWAttrs[KEYSTONE_R1_SPICOUNT] = {
#if TI_SPI_CONF_SPI0_ENABLE
    {
        .baseAddr           = SSI0_BASE,
        .intNum             = INT_SSI0_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI0,
        .defaultTxBufValue  = 0xFF,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI0_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI0_TX,
        .mosiPin            = KEYSTONE_R1_SPI0_MOSI,
        .misoPin            = KEYSTONE_R1_SPI0_MISO,
        .clkPin             = KEYSTONE_R1_SPI0_CLK,
        .csnPin             = KEYSTONE_R1_SPI0_CSN,
        .minDmaTransferSize = 10
    },
#endif
#if TI_SPI_CONF_SPI1_ENABLE
    {
        .baseAddr           = SSI1_BASE,
        .intNum             = INT_SSI1_COMB,
        .intPriority        = ~0,
        .swiPriority        = 0,
        .powerMngrId        = PowerCC26XX_PERIPH_SSI1,
        .defaultTxBufValue  = 0xFF,
        .rxChannelBitMask   = 1<<UDMA_CHAN_SSI1_RX,
        .txChannelBitMask   = 1<<UDMA_CHAN_SSI1_TX,
        .mosiPin            = KEYSTONE_R1_SPI1_MOSI,
        .misoPin            = KEYSTONE_R1_SPI1_MISO,
        .clkPin             = KEYSTONE_R1_SPI1_CLK,
        .csnPin             = KEYSTONE_R1_SPI1_CSN,
        .minDmaTransferSize = 10
    },
#endif
};

const SPI_Config SPI_config[KEYSTONE_R1_SPICOUNT] = {
#if TI_SPI_CONF_SPI0_ENABLE
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[KEYSTONE_R1_SPI0],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[KEYSTONE_R1_SPI0]
    },
#endif
#if TI_SPI_CONF_SPI1_ENABLE
    {
         .fxnTablePtr = &SPICC26XXDMA_fxnTable,
         .object      = &spiCC26XXDMAObjects[KEYSTONE_R1_SPI1],
         .hwAttrs     = &spiCC26XXDMAHWAttrs[KEYSTONE_R1_SPI1]
    },
#endif
};

const uint_least8_t SPI_count = KEYSTONE_R1_SPICOUNT;

#endif /* TI_SPI_CONF_ENABLE */


/*
 *  =============================== TRNG ===============================
 */
#include <ti/drivers/TRNG.h>
#include <ti/drivers/trng/TRNGCC26X2.h>

TRNGCC26X2_Object trngCC26X2Object[KEYSTONE_R1_TRNGCOUNT];

const TRNGCC26X2_HWAttrs trngCC26X2HWAttrs[KEYSTONE_R1_TRNGCOUNT] = {
    {
         .swiPriority = 0,
         .intPriority = ~0,
    }
};

const TRNG_Config TRNG_config[] = {
    { &trngCC26X2Object[0], &trngCC26X2HWAttrs[0] },
};

const uint8_t TRNG_count = KEYSTONE_R1_TRNGCOUNT;


/*
 *  =============================== UART ===============================
 */
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

#if TI_UART_CONF_ENABLE

UARTCC26XX_Object uartCC26XXObjects[KEYSTONE_R1_UARTCOUNT];

uint8_t uartCC26XXRingBuffer[KEYSTONE_R1_UARTCOUNT][32];

const UARTCC26XX_HWAttrsV2 uartCC26XXHWAttrs[KEYSTONE_R1_UARTCOUNT] = {
#if TI_UART_CONF_UART0_ENABLE
    {
        .baseAddr       = UART0_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_UART0,
        .intNum         = INT_UART0_COMB,
        .intPriority    = ~0,
        .swiPriority    = 0,
        .txPin          = KEYSTONE_R1_UART0_TX,
        .rxPin          = KEYSTONE_R1_UART0_RX,
        .ctsPin         = PIN_UNASSIGNED,
        .rtsPin         = PIN_UNASSIGNED,
        .ringBufPtr     = uartCC26XXRingBuffer[KEYSTONE_R1_UART0],
        .ringBufSize    = sizeof(uartCC26XXRingBuffer[KEYSTONE_R1_UART0]),
        .txIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_1_8,
        .rxIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_4_8,
        .errorFxn       = NULL
    },
#endif
#if TI_UART_CONF_UART1_ENABLE
    {
        .baseAddr       = UART1_BASE,
        .powerMngrId    = PowerCC26X2_PERIPH_UART1,
        .intNum         = INT_UART1_COMB,
        .intPriority    = ~0,
        .swiPriority    = 0,
        .txPin          = KEYSTONE_R1_UART1_TX,
        .rxPin          = KEYSTONE_R1_UART1_RX,
        .ctsPin         = PIN_UNASSIGNED,
        .rtsPin         = PIN_UNASSIGNED,
        .ringBufPtr     = uartCC26XXRingBuffer[KEYSTONE_R1_UART1],
        .ringBufSize    = sizeof(uartCC26XXRingBuffer[KEYSTONE_R1_UART1]),
        .txIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_1_8,
        .rxIntFifoThr   = UARTCC26XX_FIFO_THRESHOLD_4_8,
        .errorFxn       = NULL
    },
#endif
};

const UART_Config UART_config[KEYSTONE_R1_UARTCOUNT] = {
#if TI_UART_CONF_UART0_ENABLE
    {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects[KEYSTONE_R1_UART0],
        .hwAttrs     = &uartCC26XXHWAttrs[KEYSTONE_R1_UART0]
    },
#endif
#if TI_UART_CONF_UART1_ENABLE
    {
        .fxnTablePtr = &UARTCC26XX_fxnTable,
        .object      = &uartCC26XXObjects[KEYSTONE_R1_UART1],
        .hwAttrs     = &uartCC26XXHWAttrs[KEYSTONE_R1_UART1]
    },
#endif
};

const uint_least8_t UART_count = KEYSTONE_R1_UARTCOUNT;

#endif /* TI_UART_CONF_ENABLE */

/*
 *  =============================== UDMA ===============================
 */
#include <ti/drivers/dma/UDMACC26XX.h>

UDMACC26XX_Object udmaObjects[KEYSTONE_R1_UDMACOUNT];

const UDMACC26XX_HWAttrs udmaHWAttrs[KEYSTONE_R1_UDMACOUNT] = {
    {
        .baseAddr    = UDMA0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_UDMA,
        .intNum      = INT_DMA_ERR,
        .intPriority = ~0
    }
};

const UDMACC26XX_Config UDMACC26XX_config[KEYSTONE_R1_UDMACOUNT] = {
    {
         .object  = &udmaObjects[KEYSTONE_R1_UDMA0],
         .hwAttrs = &udmaHWAttrs[KEYSTONE_R1_UDMA0]
    },
};



/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>

WatchdogCC26XX_Object watchdogCC26XXObjects[KEYSTONE_R1_WATCHDOGCOUNT];

const WatchdogCC26XX_HWAttrs watchdogCC26XXHWAttrs[KEYSTONE_R1_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WDT_BASE,
        .reloadValue = 1000 /* Reload value in milliseconds */
    },
};

const Watchdog_Config Watchdog_config[KEYSTONE_R1_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogCC26XX_fxnTable,
        .object      = &watchdogCC26XXObjects[KEYSTONE_R1_WATCHDOG0],
        .hwAttrs     = &watchdogCC26XXHWAttrs[KEYSTONE_R1_WATCHDOG0]
    },
};

const uint_least8_t Watchdog_count = KEYSTONE_R1_WATCHDOGCOUNT;


/*
*  =============================== PDM ===============================
*/
#include "PDMCC26XX_contiki.h"

PDMCC26XX_Object PDMCC26xxObjects[KEYSTONE_R1_I2SCOUNT];

const PDMCC26XX_HWAttrs PDMCC26xxHWAttrs[KEYSTONE_R1_I2SCOUNT] = {
    {
        .micPower = PIN_UNASSIGNED,
        .taskPriority = 0,
    },
};

const PDMCC26XX_Config PDMCC26XX_config[KEYSTONE_R1_I2SCOUNT] = {
    {
        .object = &PDMCC26xxObjects[KEYSTONE_R1_I2S0],
        .hwAttrs = &PDMCC26xxHWAttrs[KEYSTONE_R1_I2S0]
    },
};


/*
*  =============================== I2S ===============================
*/
#include "PDMCC26XX_util_contiki.h"

PDMCC26XX_I2S_Object i2sCC26xxObjects[KEYSTONE_R1_I2SCOUNT];

const PDMCC26XX_I2S_HWAttrs i2sCC26xxHWAttrs[KEYSTONE_R1_I2SCOUNT] = {
    {
        .baseAddr = I2S0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2S,
        .intNum = INT_I2S_IRQ,
        .intPriority = ~0,
        .mclkPin = PIN_UNASSIGNED,
        .bclkPin = KEYSTONE_R1_I2S0_CLK,
        .wclkPin = PIN_UNASSIGNED,
        .ad0Pin = KEYSTONE_R1_I2S0_DOUT,
    },
};

const PDMCC26XX_I2S_Config PDMCC26XX_I2S_config[KEYSTONE_R1_I2SCOUNT] = {
    {
        .object = &i2sCC26xxObjects[KEYSTONE_R1_I2S0],
        .hwAttrs = &i2sCC26xxHWAttrs[KEYSTONE_R1_I2S0]
    },
};


/*
 *  Board-specific initialization function to disable external flash.
 *  This function is defined in the file KEYSTONE_R1_fxns.c
 */
extern void Board_initHook(void);

