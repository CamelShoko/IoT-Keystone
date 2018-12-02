/*
 * Copyright (c) 2018, This. Is. IoT. - https://thisisiot.io
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
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
/**
 *
 *        Any audio sensor outputing PDM format data is supported 
 *        by this driver, but it was tested with the ST MP34DT05.
 *
 *        This driver uses the TI PDM driver that utilizes the 
 *        hardware I2S to convert PDM to PCM format data in a 
 *        streaming multi-process engine.  
 *
 *        The sensors module must activate this driver with
 *        an initial and one-time call to configure(SENSORS_HW_INIT)
 *        to instantiate the PDM driver and allocate its buffers.
 *        This is normally performed by the sensors module before the
 *        processes start execution.
 *
 *        Thereafter, an application process can call SENSORS_ACTIVATE()
 *        to start streaming.  The driver will post a sensors_changed event
 *        wfor each data buffer generated.
 *        This will occur with frequency of between 2 and 4 ms depending on
 *        the selected PCM encoding (16 or 8 kHz).
 *
 *        Audio data is returned in buffers through these APIs:
 *        Either:
 *           1.  Sensor value API: 
 *                   // Get a pointer to current data buffer after sensors_changed event.
 *                   audio_sensor_buffer* buf = (audio_sensor_buffer)audio_sensor.value(AUDIO_SENSOR_GET_BUFFER);
 *                   // use buffer, e.g. memcpy(<dest>, audio_sensor_buffer->pBuffer, AUDIO_SENSOR_RET_BUF_SIZE);
                     // Then release it.  This is mandatory or the internal Queues will fill up and processing will stop.
 *                   audio_sensor.value(AUDIO_SENSOR_PUT_BUFFER);
 *
 *           2. Direct API:
 *                   audio_sensor_buffer* buf = audio_sensor_get_buffer();
 *                   audio_sensor_put_buffer();
 * \file
 *        Header file for an audio sensor delivering PDM output.
 * \author
 *        Evan Ross <evan@thisisiot.io>
 */
/*---------------------------------------------------------------------------*/
#ifndef AUDIO_SENSOR_H_
#define AUDIO_SENSOR_H_
/*---------------------------------------------------------------------------*/
#include "contiki.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
/*---------------------------------------------------------------------------*/
#define AUDIO_SENSOR_READING_ERROR  -1 /* general error */
#define AUDIO_SENSOR_NO_BUFFER      -2 /* no buffer available to value() or get_buffer() */
#define AUDIO_SENSOR_GET_BUFFER     1
#define AUDIO_SENSOR_PUT_BUFFER     2

#define AUDIO_SENSOR_SENSIVITY_DB   BOARD_AUDIO_SENSOR_SENSITIVITY_DB
/*---------------------------------------------------------------------------*/
/* Module configuration */

/* Size of the PCM data buffer returned in value(), in bytes 
 */
#ifdef AUDIO_SENSOR_CONF_RET_BUF_SIZE
#define AUDIO_SENSOR_RET_BUF_SIZE       AUDIO_SENSOR_CONF_RET_BUF_SIZE
#else
#define AUDIO_SENSOR_RET_BUF_SIZE       64  /* minimum of 64 */
#endif



/* Resultant PCM audio sample rate in kHz */
/* Supports only 8 or 16.
 * Note, workaround may be employed to double microphone bit clock and use 8 KHz sample rate
 * to achieve 16 KHz-like decimation 
 */
#ifdef AUDIO_SENSOR_CONF_SAMP_RATE_KHZ
#define AUDIO_SENSOR_SAMP_RATE_KHZ       AUDIO_SENSOR_CONF_SAMP_RATE_KHZ
#else
#define AUDIO_SENSOR_SAMP_RATE_KHZ       16 
#endif

/* Size of the array of PCM data bytes within the returned buffer is
 * the total buffer size less the header.
 */
#define AUDIO_SENSOR_PCM_DATA_SIZE      (AUDIO_SENSOR_RET_BUF_SIZE - sizeof(audio_sensor_metadata))

/*---------------------------------------------------------------------------*/
typedef enum {
  AUDIO_SENSOR_STATUS_DISABLED,
  AUDIO_SENSOR_STATUS_STANDBY,
  AUDIO_SENSOR_STATUS_STREAMING,
  AUDIO_SENSOR_STATUS_DATA_READY,
  AUDIO_SENSOR_STATUS_I2S_ERROR,
} AUDIO_SENSOR_STATUS;

/* To avoid leaking PDMCC26XX implementation details to the application, 
 * we create our own buffer structure to return the audio data in from
 * a audio_sensor.value() call.
 *
 * There is ONE instance of this, accessed through value().
 * This buffer is acquired with audio_sensor.value(AUDIO_SENSOR_GET_BUFFER)
 * This buffer must subsequently be released with  audio_sensor.value(AUDIO_SENSOR_RELEASE_BUFFER)
 */
 /*! @brief Metadata associated with an array of PCM data */
typedef struct {
    uint8_t seqNum;     /*!< Sequence number of a ::PDMCC26XX_pcmBuffer */
    int8_t si;          /*!< Step index of a ::PDMCC26XX_pcmBuffer */
    int16_t pv;         /*!< Next predicted value of a ::PDMCC26XX_pcmBuffer */
} audio_sensor_metadata;

/* Allocate the buffer according to  the "standard C flexible array member" 
 * technique.  */
typedef struct {
    audio_sensor_metadata metaData; /*!< Metadata for the buffer */
    uint8_t buffer[];              /*!< PCM data buffer, contained IN-LINE with this buffer with size AUDIO_SENSOR_PCM_DATA_SIZE */
} audio_sensor_buffer;

/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor audio_sensor;
/*---------------------------------------------------------------------------*/
#endif /* AUDIO_SENSOR_H_ */
/*---------------------------------------------------------------------------*/
