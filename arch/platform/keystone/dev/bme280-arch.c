/*
 * Copyright (c) 2018, This. Is. IoT.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 *
 * @{
 *
 * \file
 *  Architecture-specific SPI for the BME280 temperature
 *  Pressure Humidity sensor.
 *
 * \author
 *         Evan Ross <evan@thisisiot.io>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "board-peripherals.h"

#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/SPI.h>


#define BME_SPI_HANDLE  hSpiSensor

/*---------------------------------------------------------------------------*/
void
bme280_arch_i2c_init(void)
{
   /* Nothing to do here - the "sensors" SPI bus already initialized in platform */
}

/*---------------------------------------------------------------------------*/
void
bme280_arch_i2c_write_mem(uint8_t addr, uint8_t reg, uint8_t value)
{
  uint8_t txbuf[2];

  /* SPI mode: register bit 7=0 means write. */
  txbuf[0] = reg & 0x7f;
  txbuf[1] = value;
  
  PINCC26XX_setOutputValue(Board_SPI_BME280_CS, 0); /* Assert CS */

  SPI_Transaction spiTransaction;
  spiTransaction.count = sizeof(txbuf);
  spiTransaction.txBuf = (void*)txbuf;
  spiTransaction.rxBuf = NULL;
  SPI_transfer(BME_SPI_HANDLE, &spiTransaction);

  PINCC26XX_setOutputValue(Board_SPI_BME280_CS, 1); /* De-assert CS */
}
/*---------------------------------------------------------------------------*/
void
bme280_arch_i2c_read_mem(uint8_t addr, uint8_t reg, uint8_t *rxbuf, uint8_t bytes)
{
    uint8_t txbuf[1];

    /* SPI mode: register bit 7=1 means read. */
    txbuf[0] = reg |= 0x80;

    PINCC26XX_setOutputValue(Board_SPI_BME280_CS, 0); /* Assert CS */

    SPI_Transaction spiTransaction;
    spiTransaction.count = sizeof(txbuf);
    spiTransaction.txBuf = (void*)txbuf;
    spiTransaction.rxBuf = NULL;
    SPI_transfer(BME_SPI_HANDLE, &spiTransaction);

    spiTransaction.count = bytes;
    spiTransaction.txBuf = NULL;
    spiTransaction.rxBuf = (void*)rxbuf;
    SPI_transfer(BME_SPI_HANDLE, &spiTransaction);

    PINCC26XX_setOutputValue(Board_SPI_BME280_CS, 1); /* De-assert CS */

#if 0
    printf("read %d bytes: ", bytes);
    for (int i = 0; i < bytes; i++) {
        printf("%02x ", rxbuf[i]);
    }
    printf("\n");
#endif
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 */
