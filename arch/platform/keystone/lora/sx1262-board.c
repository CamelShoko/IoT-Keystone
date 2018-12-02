/*
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
* *  Neither the name of the copyright holders nor the names of
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
/*!
 * \file      sx1262dvk1cas-board.c
 *
 * \brief     Target board SX1262DVK1CAS shield driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>
#include "contiki.h"
#include "lora-utilities.h"
#include "lora-delay.h"
#include "lora-radio.h"
#include "sx126x-board.h"

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/SPI.h>


/* PIN driver handle */
static PIN_Handle hRadioPins;
static PIN_State pinState;


#define SPI_TRANSACT(arr)  do {                                                        \
    SPI_Transaction spiTransaction;                                                 \
    uint8_t         transmitBuffer[] = arr;                 \
    spiTransaction.count = sizeof(transmitBuffer);                                  \
    spiTransaction.txBuf = (void*)transmitBuffer;                                   \
    spiTransaction.rxBuf = NULL;                                                    \
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 0); /* Assert CS */           \
    SPI_transfer(hSpiInternal, &spiTransaction);                       \
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 1); /* De-assert CS */        \
} while(0)

/*!
 * Debug GPIO pins objects
 */
#if defined( USE_RADIO_DEBUG )
Gpio_t DbgPinTx;
Gpio_t DbgPinRx;
#endif

void SX126xIoInit( void )
{

    const PIN_Config pinListRadio[] =
    {
        Board_PIN_LORA_INT | PIN_INPUT_EN | PIN_PULLDOWN, /* Enable interrupt later */
        Board_PIN_LORA_BUSY | PIN_INPUT_EN,
        Board_SPI_LORA_CS | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
        Board_PIN_LORA_RST | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,

        PIN_TERMINATE
    };

    /* Acquire exclusive access to pins (through PIN API), and also
     * install handler. 
     */
    hRadioPins = PIN_open(&pinState, pinListRadio);

#if 0 /* Initialized in keystone platform. */
    GpioInit( &SX126x.Spi.Nss, RADIO_NSS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    GpioInit( &SX126x.BUSY, RADIO_BUSY, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX126x.DIO1, RADIO_DIO_1, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &DeviceSel, RADIO_DEVICE_SEL, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

#if defined( USE_RADIO_DEBUG )
    GpioInit( &DbgPinTx, RADIO_DBG_PIN_TX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &DbgPinRx, RADIO_DBG_PIN_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
#endif

}


static void callbackAdapter(PIN_Handle handle, PIN_Id pinId)
{
    DioIrqHandler* dioIrq = (DioIrqHandler*)PIN_getUserArg(handle);
    // The argument is the LoRaMac no-arg callback function to call.
    if (dioIrq) {
        dioIrq();
    }
}

void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    PIN_registerIntCb(hRadioPins, callbackAdapter);
    PIN_setUserArg(hRadioPins, (uintptr_t)dioIrq);
    PIN_setInterrupt(hRadioPins, Board_PIN_LORA_INT | PIN_IRQ_POSEDGE);
}

void SX126xIoDeInit( void )
{
#if 0 /* Initialized in keystone platform. */
    GpioInit( &SX126x.Spi.Nss, RADIO_NSS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    GpioInit( &SX126x.BUSY, RADIO_BUSY, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &SX126x.DIO1, RADIO_DIO_1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}

uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return BOARD_LORA_TXCO_WAKEUP_TIME;
}

void SX126xReset( void )
{
    DelayMs( 10 );
    //GpioInit( &SX126x.Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    PINCC26XX_setOutputValue(Board_PIN_LORA_RST, 0); /* Assert reset */
    DelayMs( 20 );
    //GpioInit( &SX126x.Reset, RADIO_RESET, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 ); // internal pull-up
    PINCC26XX_setOutputValue(Board_PIN_LORA_RST, 1); /* De-assert reset */

    DelayMs( 10 );
}

void SX126xWaitOnBusy( void )
{
    while( PINCC26XX_getInputValue(Board_PIN_LORA_BUSY ) == 1 );
}

void SX126xWakeup( void )
{
    CRITICAL_SECTION_BEGIN( );

    SPI_Transaction spiTransaction;
    uint8_t         transmitBuffer[2] = { RADIO_GET_STATUS, 0x00 };
    
    //GpioWrite( &SX126x.Spi.Nss, 0 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 0); /* Assert CS */

    //SpiInOut( &SX126x.Spi, RADIO_GET_STATUS );
    //SpiInOut( &SX126x.Spi, 0x00 );
    spiTransaction.count = sizeof(transmitBuffer);
    spiTransaction.txBuf = (void*)transmitBuffer;
    SPI_transfer(hSpiInternal, &spiTransaction);

    //GpioWrite( &SX126x.Spi.Nss, 1 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 1); /* De-assert CS */

    // Wait for chip to be ready.
    SX126xWaitOnBusy( );

    CRITICAL_SECTION_END( );
}

void SX126xWriteCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //GpioWrite( &SX126x.Spi.Nss, 0 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 0); /* Assert CS */
#if 0
    SpiInOut( &SX126x.Spi, ( uint8_t )command );

    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( &SX126x.Spi, buffer[i] );
    }
#endif
    SPI_Transaction spiTransaction;                                                 
    uint8_t         transmitBuffer[] = { command };
    spiTransaction.count = sizeof(transmitBuffer);                              
    spiTransaction.txBuf = (void*)transmitBuffer; 
    spiTransaction.rxBuf = NULL;
    SPI_transfer(hSpiInternal, &spiTransaction);

    spiTransaction.count = size;
    spiTransaction.txBuf = (void*)buffer;
    SPI_transfer(hSpiInternal, &spiTransaction);


    //GpioWrite( &SX126x.Spi.Nss, 1 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 1); /* De-assert CS */

    if( command != RADIO_SET_SLEEP )
    {
        SX126xWaitOnBusy( );
    }
}

void SX126xReadCommand( RadioCommands_t command, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //GpioWrite( &SX126x.Spi.Nss, 0 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 0); /* Assert CS */

#if 0
    SpiInOut( &SX126x.Spi, ( uint8_t )command );
    SpiInOut( &SX126x.Spi, 0x00 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }
#endif

    SPI_Transaction spiTransaction;
    uint8_t         transmitBuffer[] = { command };
    spiTransaction.count = sizeof(transmitBuffer);
    spiTransaction.txBuf = (void*)transmitBuffer;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(hSpiInternal, &spiTransaction);

    spiTransaction.count = size;
    spiTransaction.txBuf = NULL;
    spiTransaction.rxBuf = (void*)buffer;
    SPI_transfer(hSpiInternal, &spiTransaction);

    //GpioWrite( &SX126x.Spi.Nss, 1 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 1); /* De-assert CS */

    SX126xWaitOnBusy( );
}

void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //GpioWrite( &SX126x.Spi.Nss, 0 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 0); /* Assert CS */
#if 0
    SpiInOut( &SX126x.Spi, RADIO_WRITE_REGISTER );
    SpiInOut( &SX126x.Spi, ( address & 0xFF00 ) >> 8 );
    SpiInOut( &SX126x.Spi, address & 0x00FF );
    
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( &SX126x.Spi, buffer[i] );
    }
#endif

    SPI_Transaction spiTransaction;
    uint8_t         transmitBuffer[] = { RADIO_WRITE_REGISTER,  (address & 0xFF00) >> 8 , address & 0x00FF };
    spiTransaction.count = sizeof(transmitBuffer);
    spiTransaction.txBuf = (void*)transmitBuffer;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(hSpiInternal, &spiTransaction);

    spiTransaction.count = size;
    spiTransaction.txBuf = (void*)buffer;
    SPI_transfer(hSpiInternal, &spiTransaction);

    //GpioWrite( &SX126x.Spi.Nss, 1 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 1); /* De-assert CS */

    SX126xWaitOnBusy( );
}

void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters( address, &value, 1 );
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    SX126xCheckDeviceReady( );

    //GpioWrite( &SX126x.Spi.Nss, 0 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 0); /* Assert CS */

#if 0
    SpiInOut( &SX126x.Spi, RADIO_READ_REGISTER );
    SpiInOut( &SX126x.Spi, ( address & 0xFF00 ) >> 8 );
    SpiInOut( &SX126x.Spi, address & 0x00FF );
    SpiInOut( &SX126x.Spi, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }
#endif

    SPI_Transaction spiTransaction;
    /* Note host has to send NOP byte (0) after last address byte 
     * before receiving data (datasheet 13.2.2 ReadRegister Function) 
     */
    uint8_t transmitBuffer[] = { RADIO_READ_REGISTER,  (address & 0xFF00) >> 8, address & 0x00FF, 0 };
    spiTransaction.count = sizeof(transmitBuffer);
    spiTransaction.txBuf = (void*)transmitBuffer;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(hSpiInternal, &spiTransaction);

    spiTransaction.count = size;
    spiTransaction.txBuf = NULL;
    spiTransaction.rxBuf = (void*)buffer;
    SPI_transfer(hSpiInternal, &spiTransaction);

    //GpioWrite( &SX126x.Spi.Nss, 1 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 1); /* De-assert CS */

    SX126xWaitOnBusy( );
}

uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters( address, &data, 1 );
    return data;
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    //GpioWrite( &SX126x.Spi.Nss, 0 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 0); /* Assert CS */

#if 0
    SpiInOut( &SX126x.Spi, RADIO_WRITE_BUFFER );
    SpiInOut( &SX126x.Spi, offset );
    for( uint16_t i = 0; i < size; i++ )
    {
        SpiInOut( &SX126x.Spi, buffer[i] );
    }
#endif

    SPI_Transaction spiTransaction;
    uint8_t         transmitBuffer[] = { RADIO_WRITE_BUFFER,  offset };
    spiTransaction.count = sizeof(transmitBuffer);
    spiTransaction.txBuf = (void*)transmitBuffer;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(hSpiInternal, &spiTransaction);

    spiTransaction.count = size;
    spiTransaction.txBuf = (void*)buffer;
    SPI_transfer(hSpiInternal, &spiTransaction);

    //GpioWrite( &SX126x.Spi.Nss, 1 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 1); /* De-assert CS */

    SX126xWaitOnBusy( );
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    SX126xCheckDeviceReady( );

    //GpioWrite( &SX126x.Spi.Nss, 0 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 0); /* Assert CS */

#if 0
    SpiInOut( &SX126x.Spi, RADIO_READ_BUFFER );
    SpiInOut( &SX126x.Spi, offset );
    SpiInOut( &SX126x.Spi, 0 );
    for( uint16_t i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut( &SX126x.Spi, 0 );
    }
#endif
    SPI_Transaction spiTransaction;
    /* Note host has to send NOP byte (0) after last address byte
    * before receiving data (datasheet 13.2.2 ReadRegister Function)
    */
    uint8_t transmitBuffer[] = { RADIO_READ_REGISTER,  offset, 0 };
    spiTransaction.count = sizeof(transmitBuffer);
    spiTransaction.txBuf = (void*)transmitBuffer;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(hSpiInternal, &spiTransaction);

    spiTransaction.count = size;
    spiTransaction.txBuf = NULL;
    spiTransaction.rxBuf = (void*)buffer;
    SPI_transfer(hSpiInternal, &spiTransaction);

    //GpioWrite( &SX126x.Spi.Nss, 1 );
    PINCC26XX_setOutputValue(Board_SPI_LORA_CS, 1); /* De-assert CS */

    SX126xWaitOnBusy( );
}

void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

uint8_t SX126xGetDeviceId( void )
{
#if 0
    if( GpioRead( &DeviceSel ) == 1 )
    {
        return SX1261;
    }
    else
    {
        return SX1262;
    }
#endif
    return SX1262;
}

void SX126xAntSwOn( void )
{
    //GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
    PINCC26XX_setOutputValue(Board_RF_SUB1GHZ, 1);
}

void SX126xAntSwOff( void )
{
    //GpioInit( &AntPow, RADIO_ANT_SWITCH_POWER, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    PINCC26XX_setOutputValue(Board_RF_SUB1GHZ, 0);
}

bool SX126xCheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

#if defined( USE_RADIO_DEBUG )
void SX126xDbgPinTxWrite( uint8_t state )
{
    GpioWrite( &DbgPinTx, state );
}

void SX126xDbgPinRxWrite( uint8_t state )
{
    GpioWrite( &DbgPinRx, state );
}
#endif
