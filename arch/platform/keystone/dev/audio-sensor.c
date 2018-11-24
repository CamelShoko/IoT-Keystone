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
 *        The audio sensor uses the TI CC26XX PDM and I2S drivers to convert
 *        a microphone's PDM output into PCM samples with optional compression
 *        and configurable sample rate.  Leveraging compression at this level
 *        is highly recommended as the sample data eventually has to move
 *        over a lossy and low datarate mesh or LoRaWAN link!  The less data to move
 *        the better.
 *
 *        The PDM driver operates under a startStream() / stopStream() framework.
 *        After it is opened once at platform startup through configure(SENSORS_HW_INIT) 
 *        (which allocates a fixed set of I2S buffers from the heap), the 
 *        SENSORS_ACTIVATE() process will invoke PDMCC26XX_startStream().
 *        
 *        The PDM driver has an internal Contiki process that performs the
 *        PDM to PCM conversion.  For each converted buffer, the PDM driver's internal process
 *        will invoke a callback supplied in this sensor driver.  In the callback
 *        we then post the "sensors_changed" event.  At most ONE user process can then
 *        use the PCM data buffer through the value() API.
 *
 *        Some digital microphones such as the MP34DT05 have a startup delay that
 *        transitions the device from sleep to normal mode on first clock edge.
 *        To account for this, the PDM parameter "startupDelayWithClockInSamples" is
 *        set appropriately.  For reference, the MP34DT05 has a 10 ms startup delay.
 *
 * \file
 *        Driver for an audio sensor delivering PDM output.
 * \author
 *        Evan Ross <evan@thisisiot.io>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "sys/ctimer.h"
/*---------------------------------------------------------------------------*/
#include "board-conf.h"
#include "audio-sensor.h"
/*---------------------------------------------------------------------------*/
#include <Board.h>
#include "PDMCC26XX_contiki.h"
/*---------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 1 
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
typedef struct {
  volatile AUDIO_SENSOR_STATUS status;
} AUDIO_SENSOR_Object;

static AUDIO_SENSOR_Object audio_sensor_obj;

/*---------------------------------------------------------------------------*/
static PDMCC26XX_Handle pdm_handle;
/*---------------------------------------------------------------------------*/

static void
bufRdy_callback(PDMCC26XX_Handle handle, PDMCC26XX_StreamNotification *streamNotification);

static void* malloc_fxn(size_t size) {
    return malloc(size);
}

static void free_fxn(void* ptr, size_t size) {
    free(ptr);
}

/*---------------------------------------------------------------------------*/
/**
 * \brief   Initialize the audio sensor driver.
 * \return  true if PDM operation successful; else, return false.
 */
static bool
sensor_init(void)
{
  if(pdm_handle) {
    return true;
  }

  PDMCC26XX_Params pdmParams;
  PDMCC26XX_Contiki_Params_init(&pdmParams);

  pdmParams.callbackFxn = bufRdy_callback;
  pdmParams.micPowerActiveHigh = true;
  pdmParams.applyCompression = true;
  pdmParams.retBufSizeInBytes = AUDIO_SENSOR_RET_BUF_SIZE;
  pdmParams.mallocFxn = &malloc_fxn;
  pdmParams.freeFxn = &free_fxn;
  
  // Configure the params to use 8kHz PCM output (4 ms buffer period instead of 2 ms)
  pdmParams.pcmSampleRate = PDMCC26XX_PCM_SAMPLE_RATE_8K;

  pdm_handle = PDMCC26XX_Contiki_open(&pdmParams);
  if(pdm_handle == NULL) {
    return false;
  }

  audio_sensor_obj.status = AUDIO_SENSOR_STATUS_DISABLED;

  return true;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief         Enable/disable audio streaming
 * \param enable  Start streaming if true; else, stop streaming.
 * \return        true if operation succeeded, false if error.
 */
static bool
sensor_enable(bool enable)
{
    /* Don't do anything if we couldn't init the sensor through configure(). */
    if (audio_sensor_obj.status == AUDIO_SENSOR_STATUS_DISABLED) {
       return false;
    }

    if (enable) {
        if (PDMCC26XX_Contiki_startStream(pdm_handle)) {
            audio_sensor_obj.status = AUDIO_SENSOR_STATUS_STREAMING;
            return true;
        } 
    }
    else {
        if (PDMCC26XX_Contiki_stopStream(pdm_handle)) {
            audio_sensor_obj.status = AUDIO_SENSOR_STATUS_STANDBY;
            return true;
        }
    }

    return false;

}
/*---------------------------------------------------------------------------*/
/**
 * \brief  Callback when PDM driver has a PCM buffer ready.
 */
static void
bufRdy_callback(PDMCC26XX_Handle handle, PDMCC26XX_StreamNotification *streamNotification)
{
    switch (streamNotification->status)
    {
    case PDMCC26XX_STREAM_BLOCK_READY:
    case PDMCC26XX_STREAM_BLOCK_READY_BUT_PDM_OVERFLOW:
    case PDMCC26XX_STREAM_FAILED_TO_STOP:
    case PDMCC26XX_STREAM_STOPPING:
        /* All cases above result in a buffer available so we post it. */
        /* User is required to access buffer with audio_sensor.value() */
        audio_sensor_obj.status = AUDIO_SENSOR_STATUS_DATA_READY;
        sensors_changed(&audio_sensor);
        break;
    case PDMCC26XX_STREAM_ERROR:
        audio_sensor_obj.status = AUDIO_SENSOR_STATUS_I2S_ERROR;
        break;
    default:
        /* Other non-error conditions indicate driver is heading to, or in, 
         * stopped streaming mode which we'll call STANDBY.
         */
        audio_sensor_obj.status = AUDIO_SENSOR_STATUS_STANDBY;
        break;
    }


}
/*---------------------------------------------------------------------------*/
/**
 * \brief       Returns address of a data buffer containing available audio data.
 * \param type  Get or Put command
 * \return      Pointer to audio_sensor_buffer, or AUDIO_SENSOR_READING_ERROR if
 *              state is incorrect or some other error.
 */
static int
value(int type)
{
  /* hold on to this last buffer request for subsequent freeing as
   * required by PDM driver.
   */
  static PDMCC26XX_BufferRequest bufferRequest = { 0 };

  /* return error unless explicilty set successful */
  int rv = AUDIO_SENSOR_READING_ERROR;

  if(audio_sensor_obj.status != AUDIO_SENSOR_STATUS_DATA_READY) {
      return rv;
  }

  switch(type)
  {
    case AUDIO_SENSOR_GET_BUFFER:
        if (bufferRequest.buffer == NULL) { /* must not be a buffer pending put */
            if (PDMCC26XX_Contiki_requestBuffer(pdm_handle, &bufferRequest)) {
                rv = (int)bufferRequest.buffer;
            }
            else {
                rv = AUDIO_SENSOR_NO_BUFFER;
            }
        }
        break;
    case AUDIO_SENSOR_PUT_BUFFER:
        if (bufferRequest.buffer != NULL) { /* must be a buffer pending put */
            /* release using same allocation mechanism supplied at open */
            free(bufferRequest.buffer);
            bufferRequest.buffer = NULL;
            rv = 0;
        }
        break;
    default:
        break;

  }

  return rv;

}
/*---------------------------------------------------------------------------*/
/**
 * \brief         Configuration function for the OPT3001 sensor.
 * \param type    Activate, enable or disable the sensor. See below.
 * \param enable  Enable or disable sensor.
 *
 *                When type == SENSORS_HW_INIT we open the PDM and I2S drivers.
 *                When type == SENSORS_ACTIVE and enable==1 we enable the sensor.
 *                When type == SENSORS_ACTIVE and enable==0 we disable the sensor.
 */
static int
configure(int type, int enable)
{
  int rv = 0;
  switch(type) {
  case SENSORS_HW_INIT:
    PRINTF("Audio sensor init\n");
    if(sensor_init()) {
      audio_sensor_obj.status = AUDIO_SENSOR_STATUS_STANDBY;
      PRINTF("Audio sensor init [success]\n");
    } else {
      audio_sensor_obj.status = AUDIO_SENSOR_STATUS_DISABLED;
      rv = AUDIO_SENSOR_READING_ERROR;
      PRINTF("Audio sensor init [FAIL]\n");
    }
    break;

  /* Case when SENSORS_ACTIVATE() is called. */
  case SENSORS_ACTIVE:
    if(enable) {
      if (!sensor_enable(true)) {
        rv = AUDIO_SENSOR_READING_ERROR;
      }
    } else {
      if (!sensor_enable(false)) {
        rv = AUDIO_SENSOR_READING_ERROR;
      }
    }
    break;

  default:
    break;
  }

  return rv;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief       Returns the status of the sensor.
 * \param type  Ignored.
 * \return      The status of the sensor.
 */
static int
status(int type)
{
  /* Unused args */
  (void)type;

  return audio_sensor_obj.status;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(audio_sensor, "AUDIO", value, configure, status);
/*---------------------------------------------------------------------------*/
