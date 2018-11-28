/*
 * Copyright (c) 2017-2018, Texas Instruments Incorporated
 * Copyright (c) 2018, This. Is. IoT. - https://thisisiot.io
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
/** ===========================================================================
 *  @file       KEYSTONE_R1.h
 *
 *  @brief      KEYSTONE_R1 Board Specific header file.
 *
 *  The KEYSTONE_R1 header file should be included in an application as
 *  follows:
 *  @code
 *  #include "KEYSTONE_R1.h"
 *  @endcode
 *
 *  ===========================================================================
 */
#ifndef __KEYSTONE_R1_BOARD_H__
#define __KEYSTONE_R1_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "contiki-conf.h"

/* Includes */
#include <ti/drivers/I2C.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/PIN.h>
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/ioc.h)

/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Singleton instance handles for device drivers SPI, I2C, I2S */
extern SPI_Handle hSpiInternal;
extern SPI_Handle hSpiSensor;
extern I2C_Handle hI2cSensor;

/* Defines */
#define KEYSTONE_R1

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>         <pin mapping>   <comments>
 */

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                  <pin mapping>
 */
/* Analog Capable DIOs */
#define KEYSTONE_R1_DIO23_ANALOG          IOID_23
#define KEYSTONE_R1_DIO24_ANALOG          IOID_24
#define KEYSTONE_R1_DIO25_ANALOG          IOID_25
#define KEYSTONE_R1_DIO26_ANALOG          IOID_26
#define KEYSTONE_R1_DIO27_ANALOG          IOID_27
#define KEYSTONE_R1_DIO28_ANALOG          IOID_28
#define KEYSTONE_R1_DIO29_ANALOG          IOID_29

/* Antenna switch */
/* PE4259 has both CTRL and /CTRL connected to DIO.
 * Configured in single-pin mode, /CTRL should be connected to Vdd.
 * In the GPIO config table BoardGpioInitTable we take care of that.
 * The reason why /CTRL is connected to GPIO and not Vdd is that we
 * might want to turn it off when the system is sleeping to stop 
 * any constant drain that might be occuring (to be measured)
 */
/* LOW  = RF2 CC1352 Sub-GHz
 * HIGH = RF1 SX1262 LoRa 
 */
#define KEYSTONE_R1_RF_SUB1GHZ            IOID_6 /* CTRL (control pin) */
#define KEYSTONE_R1_RF_SUB1GHZ_NCTRL      IOID_7 /* /CTRL (connect to Vdd) */

/* Digital IOs */
#define KEYSTONE_R1_DIO12                 IOID_12
#define KEYSTONE_R1_DIO15                 IOID_15
#define KEYSTONE_R1_DIO16_TDO             IOID_16
#define KEYSTONE_R1_DIO17_TDI             IOID_17
#define KEYSTONE_R1_DIO21                 IOID_21
#define KEYSTONE_R1_DIO22                 IOID_22

/* Discrete Inputs */
#if 0 /* no buttons */
#define KEYSTONE_R1_PIN_BTN1              IOID_15
#define KEYSTONE_R1_PIN_BTN2              IOID_14
#endif

/* GPIO */
#define KEYSTONE_R1_PIN_LORA_INT          IOID_5  /* SX1262 interrupt (input) */
#define KEYSTONE_R1_PIN_LORA_RST          IOID_9  /* SX1262 reset (output) */
#define KEYSTONE_R1_PIN_LORA_RST_ON       0
#define KEYSTONE_R1_PIN_LORA_RST_OFF      1
#define KEYSTONE_R1_PIN_LORA_BUSY         IOID_13 /* SX1262 busy (input) */
#define KEYSTONE_R1_PIN_FLASH_EN          IOID_15 /* Flash enable (output) */
#define KEYSTONE_R1_PIN_FLASH_EN_ON       0
#define KEYSTONE_R1_PIN_FLASH_EN_OFF      1
#define KEYSTONE_R1_PIN_HALL              IOID_26 /* Hall sensor digital input */
#define KEYSTONE_R1_PIN_IMU_INT           IOID_27 /* ICM20948 interrupt (input) */
#define KEYSTONE_R1_PIN_ALS_INT           IOID_28 /* OPT3001 interrupt (input) */

/* I2C */
#define KEYSTONE_R1_I2C0_SCL0             IOID_29
#define KEYSTONE_R1_I2C0_SDA0             IOID_30

/* I2S */
#define KEYSTONE_R1_I2S0_CLK              IOID_3
#define KEYSTONE_R1_I2S0_DOUT             IOID_4

/* LEDs */
#define KEYSTONE_R1_PIN_LED_ON            1
#define KEYSTONE_R1_PIN_LED_OFF           0
#define KEYSTONE_R1_PIN_RLED              IOID_16
#define KEYSTONE_R1_PIN_GLED              IOID_17

/* PWM Outputs */
#define KEYSTONE_R1_PWMPIN0               KEYSTONE_R1_PIN_RLED
#define KEYSTONE_R1_PWMPIN1               KEYSTONE_R1_PIN_GLED
#define KEYSTONE_R1_PWMPIN2               PIN_UNASSIGNED
#define KEYSTONE_R1_PWMPIN3               PIN_UNASSIGNED
#define KEYSTONE_R1_PWMPIN4               PIN_UNASSIGNED
#define KEYSTONE_R1_PWMPIN5               PIN_UNASSIGNED
#define KEYSTONE_R1_PWMPIN6               PIN_UNASSIGNED
#define KEYSTONE_R1_PWMPIN7               PIN_UNASSIGNED

/* SPI */
#define KEYSTONE_R1_SPI_FLASH_CS          IOID_14
#define KEYSTONE_R1_FLASH_CS_ON           0
#define KEYSTONE_R1_FLASH_CS_OFF          1
#define KEYSTONE_R1_SPI_LORA_CS           IOID_8
#define KEYSTONE_R1_LORA_CS_ON            0
#define KEYSTONE_R1_LORA_CS_OFF           1
#define KEYSTONE_R1_SPI_BME280_CS         IOID_25
#define KEYSTONE_R1_BME280_CS_ON          0
#define KEYSTONE_R1_BME280_CS_OFF         1
#define KEYSTONE_R1_SPI_IMU_CS            IOID_24
#define KEYSTONE_R1_IMU_CS_ON             0
#define KEYSTONE_R1_IMU_CS_OFF            1
#define KEYSTONE_R1_SPI_SENSOR3_CS        IOID_23

/* SPI Board */
#define KEYSTONE_R1_SPI0_MISO             IOID_12         
#define KEYSTONE_R1_SPI0_MOSI             IOID_11         
#define KEYSTONE_R1_SPI0_CLK              IOID_10         
#define KEYSTONE_R1_SPI0_CSN              PIN_UNASSIGNED
#define KEYSTONE_R1_SPI1_MISO             IOID_20
#define KEYSTONE_R1_SPI1_MOSI             IOID_19
#define KEYSTONE_R1_SPI1_CLK              IOID_18
#define KEYSTONE_R1_SPI1_CSN              PIN_UNASSIGNED

/* UART Board */
#define KEYSTONE_R1_UART0_RX              IOID_22         /* RXD */
#define KEYSTONE_R1_UART0_TX              IOID_21         /* TXD */
#define KEYSTONE_R1_UART0_CTS             PIN_UNASSIGNED 
#define KEYSTONE_R1_UART0_RTS             PIN_UNASSIGNED 
#define KEYSTONE_R1_UART1_RX              PIN_UNASSIGNED
#define KEYSTONE_R1_UART1_TX              PIN_UNASSIGNED
#define KEYSTONE_R1_UART1_CTS             PIN_UNASSIGNED
#define KEYSTONE_R1_UART1_RTS             PIN_UNASSIGNED
/* For backward compatibility */
#define KEYSTONE_R1_UART_RX               KEYSTONE_R1_UART0_RX
#define KEYSTONE_R1_UART_TX               KEYSTONE_R1_UART0_TX
#define KEYSTONE_R1_UART_CTS              KEYSTONE_R1_UART0_CTS
#define KEYSTONE_R1_UART_RTS              KEYSTONE_R1_UART0_RTS

#define KEYSTONE_R1_LORA_TXCO_WAKEUP_TIME 0

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void KEYSTONE_R1_initGeneral(void);

/*!
 *  @brief  Shut down the external flash present on the board files
 *
 *  This function bitbangs the SPI sequence necessary to turn off
 *  the external flash on LaunchPads.
 */
void KEYSTONE_R1_shutDownExtFlash(void);

/*!
 *  @brief  Wake up the external flash present on the board files
 *
 *  This function toggles the chip select for the amount of time needed
 *  to wake the chip up.
 */
void KEYSTONE_R1_wakeUpExtFlash(void);

/*!
 *  @def    KEYSTONE_R1_ADCBufName
 *  @brief  Enum of ADCs
 */
typedef enum KEYSTONE_R1_ADCBufName {
    KEYSTONE_R1_ADCBUF0 = 0,

    KEYSTONE_R1_ADCBUFCOUNT
} KEYSTONE_R1_ADCBufName;

/*!
 *  @def    KEYSTONE_R1_ADCBuf0ChannelName
 *  @brief  Enum of ADCBuf channels
 */
typedef enum KEYSTONE_R1_ADCBuf0ChannelName {
    KEYSTONE_R1_ADCBUF0CHANNEL0 = 0,
    KEYSTONE_R1_ADCBUF0CHANNEL1,
    KEYSTONE_R1_ADCBUF0CHANNEL2,
    KEYSTONE_R1_ADCBUF0CHANNEL3,
    KEYSTONE_R1_ADCBUF0CHANNEL4,
    KEYSTONE_R1_ADCBUF0CHANNEL5,
    KEYSTONE_R1_ADCBUF0CHANNEL6,
    KEYSTONE_R1_ADCBUF0CHANNELVDDS,
    KEYSTONE_R1_ADCBUF0CHANNELDCOUPL,
    KEYSTONE_R1_ADCBUF0CHANNELVSS,

    KEYSTONE_R1_ADCBUF0CHANNELCOUNT
} KEYSTONE_R1_ADCBuf0ChannelName;

/*!
 *  @def    KEYSTONE_R1_ADCName
 *  @brief  Enum of ADCs
 */
typedef enum KEYSTONE_R1_ADCName {
    KEYSTONE_R1_ADC0 = 0,
    KEYSTONE_R1_ADC1,
    KEYSTONE_R1_ADC2,
    KEYSTONE_R1_ADC3,
    KEYSTONE_R1_ADC4,
    KEYSTONE_R1_ADC5,
    KEYSTONE_R1_ADC6,
    KEYSTONE_R1_ADCDCOUPL,
    KEYSTONE_R1_ADCVSS,
    KEYSTONE_R1_ADCVDDS,

    KEYSTONE_R1_ADCCOUNT
} KEYSTONE_R1_ADCName;

/*!
 *  @def    KEYSTONE_R1_ECDHName
 *  @brief  Enum of ECDH names
 */
typedef enum KEYSTONE_R1_ECDHName {
    KEYSTONE_R1_ECDH0 = 0,

    KEYSTONE_R1_ECDHCOUNT
} KEYSTONE_R1_ECDHName;

/*!
 *  @def    KEYSTONE_R1_ECDSAName
 *  @brief  Enum of ECDSA names
 */
typedef enum KEYSTONE_R1_ECDSAName {
    KEYSTONE_R1_ECDSA0 = 0,

    KEYSTONE_R1_ECDSACOUNT
} KEYSTONE_R1_ECDSAName;

/*!
 *  @def    KEYSTONE_R1_ECJPAKEName
 *  @brief  Enum of ECJPAKE names
 */
typedef enum KEYSTONE_R1_ECJPAKEName {
    KEYSTONE_R1_ECJPAKE0 = 0,

    KEYSTONE_R1_ECJPAKECOUNT
} KEYSTONE_R1_ECJPAKEName;

/*!
 *  @def    KEYSTONE_R1_AESCCMName
 *  @brief  Enum of AESCCM names
 */
typedef enum KEYSTONE_R1_AESCCMName {
    KEYSTONE_R1_AESCCM0 = 0,

    KEYSTONE_R1_AESCCMCOUNT
} KEYSTONE_R1_AESCCMName;

/*!
 *  @def    KEYSTONE_R1_AESECBName
 *  @brief  Enum of AESECB names
 */
typedef enum KEYSTONE_R1_AESECBName {
    KEYSTONE_R1_AESECB0 = 0,

    KEYSTONE_R1_AESECBCOUNT
} KEYSTONE_R1_AESECBName;

/*!
 *  @def    KEYSTONE_R1_SHA2Name
 *  @brief  Enum of SHA2 names
 */
typedef enum KEYSTONE_R1_SHA2Name {
    KEYSTONE_R1_SHA20 = 0,

    KEYSTONE_R1_SHA2COUNT
} KEYSTONE_R1_SHA2Name;

/*!
 *  @def    KEYSTONE_R1_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum KEYSTONE_R1_GPTimerName {
    KEYSTONE_R1_GPTIMER0A = 0,
    KEYSTONE_R1_GPTIMER0B,
    KEYSTONE_R1_GPTIMER1A,
    KEYSTONE_R1_GPTIMER1B,
    KEYSTONE_R1_GPTIMER2A,
    KEYSTONE_R1_GPTIMER2B,
    KEYSTONE_R1_GPTIMER3A,
    KEYSTONE_R1_GPTIMER3B,

    KEYSTONE_R1_GPTIMERPARTSCOUNT
} KEYSTONE_R1_GPTimerName;

/*!
 *  @def    KEYSTONE_R1_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum KEYSTONE_R1_GPTimers {
    KEYSTONE_R1_GPTIMER0 = 0,
    KEYSTONE_R1_GPTIMER1,
    KEYSTONE_R1_GPTIMER2,
    KEYSTONE_R1_GPTIMER3,

    KEYSTONE_R1_GPTIMERCOUNT
} KEYSTONE_R1_GPTimers;

/*!
 *  @def    KEYSTONE_R1_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum KEYSTONE_R1_I2CName {
#if TI_I2C_CONF_I2C0_ENABLE
    KEYSTONE_R1_I2C0 = 0,
#endif

    KEYSTONE_R1_I2CCOUNT
} KEYSTONE_R1_I2CName;

/*!
 *  @def    KEYSTONE_R1_NVSName
 *  @brief  Enum of NVS names
 */
typedef enum KEYSTONE_R1_NVSName {
#if TI_NVS_CONF_NVS_INTERNAL_ENABLE
    KEYSTONE_R1_NVSCC26XX0 = 0,
#endif
#if TI_NVS_CONF_NVS_EXTERNAL_ENABLE
    KEYSTONE_R1_NVSSPI25X0,
#endif

    KEYSTONE_R1_NVSCOUNT
} KEYSTONE_R1_NVSName;

/*!
 *  @def    KEYSTONE_R1_PWMName
 *  @brief  Enum of PWM outputs
 */
typedef enum KEYSTONE_R1_PWMName {
    KEYSTONE_R1_PWM0 = 0,
    KEYSTONE_R1_PWM1,
    KEYSTONE_R1_PWM2,
    KEYSTONE_R1_PWM3,
    KEYSTONE_R1_PWM4,
    KEYSTONE_R1_PWM5,
    KEYSTONE_R1_PWM6,
    KEYSTONE_R1_PWM7,

    KEYSTONE_R1_PWMCOUNT
} KEYSTONE_R1_PWMName;

/*!
 *  @def    KEYSTONE_R1_SDName
 *  @brief  Enum of SD names
 */
typedef enum KEYSTONE_R1_SDName {
    KEYSTONE_R1_SDSPI0 = 0,

    KEYSTONE_R1_SDCOUNT
} KEYSTONE_R1_SDName;

/*!
 *  @def    KEYSTONE_R1_SPIName
 *  @brief  Enum of SPI names
 */
typedef enum KEYSTONE_R1_SPIName {
#if TI_SPI_CONF_SPI0_ENABLE
    KEYSTONE_R1_SPI0 = 0,
#endif
#if TI_SPI_CONF_SPI1_ENABLE
    KEYSTONE_R1_SPI1,
#endif

    KEYSTONE_R1_SPICOUNT
} KEYSTONE_R1_SPIName;

/*!
 *  @def    KEYSTONE_R1_TRNGName
 *  @brief  Enum of TRNGs
 */
typedef enum KEYSTONE_R1_TRNGName {
    KEYSTONE_R1_TRNG0 = 0,

    KEYSTONE_R1_TRNGCOUNT
} KEYSTONE_R1_TRNGName;

/*!
 *  @def    KEYSTONE_R1_UARTName
 *  @brief  Enum of UARTs
 */
typedef enum KEYSTONE_R1_UARTName {
#if TI_UART_CONF_UART0_ENABLE
    KEYSTONE_R1_UART0 = 0,
#endif
#if TI_UART_CONF_UART1_ENABLE
    KEYSTONE_R1_UART1,
#endif

    KEYSTONE_R1_UARTCOUNT
} KEYSTONE_R1_UARTName;

/*!
 *  @def    KEYSTONE_R1_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum KEYSTONE_R1_UDMAName {
    KEYSTONE_R1_UDMA0 = 0,

    KEYSTONE_R1_UDMACOUNT
} KEYSTONE_R1_UDMAName;

/*!
 *  @def    KEYSTONE_R1_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum KEYSTONE_R1_WatchdogName {
    KEYSTONE_R1_WATCHDOG0 = 0,

    KEYSTONE_R1_WATCHDOGCOUNT
} KEYSTONE_R1_WatchdogName;

/*!
*  @def    KEYSTONE_R1_I2SName
*  @brief  Enum of I2S names
*/
typedef enum KEYSTONE_R1_I2SName {
    KEYSTONE_R1_I2S0 = 0,

    KEYSTONE_R1_I2SCOUNT
} KEYSTONE_R1_I2SName;

#ifdef __cplusplus
}
#endif

#endif /* __KEYSTONE_R1_BOARD_H__ */
