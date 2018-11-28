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
/**
* \file
*         Gathers data from the Keystone platform sensors and
*         submits the data to a LoRaWAN application server.
*
*         This example uses the direct APIs of the LoRaMacApp
*         class A adapter.
*
* \author Evan Ross <evan@thisisiot.io>
*/

#include "contiki.h"
#include "sys/log.h"
#include "dev/leds.h"
#include <stdbool.h>

/* Sensor headers */
#include "board-peripherals.h"

#define DEBUG DEBUG_PRINT
#include "net/ipv6/uip-debug.h"

/*---------------------------------------------------------------------------*/

static bool bme_available = false;
static bool opt3001_available = false;

/*---------------------------------------------------------------------------*/
PROCESS(data_logger_process, "data logger");
AUTOSTART_PROCESSES(&data_logger_process);

/*---------------------------------------------------------------------------*/
static void
get_light_reading()
{
    int value = opt_3001_sensor.value(0);
    if (value != OPT_3001_READING_ERROR) {
        printf("OPT: Light=%d.%02d lux\n", value / 100, value % 100);
    }
    else {
        printf("OPT: Light Read Error\n");
    }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(data_logger_process, ev, data)
{
    PROCESS_BEGIN();

    static struct etimer led_timer;
    static struct etimer sample_timer;

    PRINTF("Starting Data Logger DEMO\n");
    
    etimer_set(&sample_timer, CLOCK_SECOND * 5);

    /* start with 1 Hz toggle. On any sensor error set to 2 Hz toggle */
    etimer_set(&led_timer, CLOCK_SECOND * 1);

    /* start audio streaming from the microphone */
    SENSORS_ACTIVATE(audio_sensor);

    while (1) {
        PROCESS_YIELD();

        if (ev == PROCESS_EVENT_TIMER) {
            if (etimer_expired(&led_timer)) {
                etimer_reset(&led_timer);
                leds_toggle(LEDS_ALL); /* flash the Keystone board user LEDs */
            }
            if (etimer_expired(&sample_timer)) {
                etimer_reset(&sample_timer);
                PRINTF("[time %02lu:%02lu:%02lu.%03lu] Sampling sensors ---------- \n", 
                    (clock_time() / CLOCK_SECOND / 3600),
                    (clock_time() / CLOCK_SECOND / 60) % 60,
                    (clock_time() / CLOCK_SECOND) % 60,
                    ((clock_time() * 1000) / CLOCK_SECOND) % 1000);

                /*-----BME280---------------------------------*/

                if (!bme_available) {
                    /* Use Contiki-NG BME driver directly, as the sensor interface is poorly designed. */
                    int result = bme280_init(BME280_MODE_WEATHER);
                    if (result) {
                        bme_available = true;
                        PRINTF("BME280 found.\n");
                    }
                    else {
                        PRINTF("BME280 not found.\n");
                        etimer_set(&led_timer, CLOCK_SECOND / 2);
                    }
                }

                if (bme_available) {
                    bme280_read(BME280_MODE_WEATHER);
                    /* Access BME measurement structure directly. */
                    //int temp = bme280_mea.t_overscale100 / 100;
                    //int hum = bme280_mea.h_overscale1024 >> 10;
                    //int pres = bme280_mea.p_overscale256 / 256;
                    PRINTF("T_BME280=%lu.%lu C", bme280_mea.t_overscale100 / 100, bme280_mea.t_overscale100 % 100);
                    PRINTF(" RH_BME280=%lu.%lu %%RH", bme280_mea.h_overscale1024 / 1024, bme280_mea.h_overscale1024 % 1024);
                    PRINTF(" P_BME280=%lu.%lu Pa\n", bme280_mea.p_overscale256 / 256, bme280_mea.p_overscale256 % 256);
                }

                /*-----OPT3001---------------------------------*/

                if (!opt3001_available) {
                    /* Identify the sensor. */
                    int result = opt_3001_sensor.status(OPT_3001_IDENTIFY);
                    if (result) {
                        opt3001_available = true;
                        PRINTF("opt3001 found. mfg=%04x dev=%04x\n",
                            (result >> 16) & 0xffff, result & 0xffff);
                    }
                    else {
                        PRINTF("opt3001 not found. result=%d\n", result);
                        etimer_set(&led_timer, CLOCK_SECOND / 2);
                    }
                }

                if (opt3001_available) {
                    /* Trigger a read cycle.  The result will come back shortly
                    * as an event.
                    */
                    SENSORS_ACTIVATE(opt_3001_sensor);
                }
            }
        } else if (ev == sensors_event) {
            if (data == &opt_3001_sensor) {
                get_light_reading();
            }
            else if (data == &audio_sensor) {
                /* buffer is ready */
                /* in case we were latent somehow ?? read and process buffers until
                 * there are no more. */
                int result = audio_sensor.value(AUDIO_SENSOR_GET_BUFFER);
                while (result != AUDIO_SENSOR_READING_ERROR &&
                    result != AUDIO_SENSOR_NO_BUFFER)
                {
                    audio_sensor_buffer* buf = (audio_sensor_buffer*)result;

                    static int count = 0;
                    // do something with it
                    //...
                    if ((++count % 100) == 0) {
                        /* Slow down console output to avoid dropping buffers.
                         * Need to increase the number of buffers in the driver since this
                         * PRINTF can cause relatively signficant delay (as buffers arrive sub 10 ms).
                         */
                        PRINTF("audio buffer: %d (count=%d)\n", buf->metaData.seqNum, count);
                    }
                    // Put it back
                    audio_sensor.value(AUDIO_SENSOR_PUT_BUFFER);

                    // See if there is another one.
                    result = audio_sensor.value(AUDIO_SENSOR_GET_BUFFER);
                }
            }
            
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
