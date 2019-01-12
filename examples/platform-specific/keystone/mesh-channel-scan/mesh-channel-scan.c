/*
* Copyright (c) 2018, THIS. IS. IoT. - https://thisisiot.io
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
*        Simple channel RSSI scanner. 
*        Prints results to a graph like this (129 channel 915 band shown)
*
-----------------------------------------------------------------
-08:
-16:
-24:
-32:
-40:
-48:
-56:
-64:
-72:
-80:
-88:
-96:
-104:
-112:
-120: **      *     ***   *      **   *       *     ** * ** *        *
-128: *****************************************************************
-----------------------------------------------------------------
0                                                               64
----------------------------------------------------------------
-08:
-16:
-24:
-32:
-40:
-48:
-56:
-64:
-72:
-80:
-88:
-96:
-104:
-112:
-120:             *     *       * **     *  *         *        *
-128: ****************************************************************
----------------------------------------------------------------
65                                                              128


* \author Evan Ross <evan@thisisiot.io>
*/

#include "contiki.h"
#include "sys/log.h"
#include "dev/leds.h"
#include "rf/dot-15-4g.h"
#include "dev/radio.h"
#include "net/netstack.h"

//#define DEBUG DEBUG_NONE
#define DEBUG DEBUG_NONE
#include "net/ipv6/uip-debug.h"


#define NUM_CHANNELS (DOT_15_4G_CHAN_MAX - DOT_15_4G_CHAN_MIN + 1)

#define NO_RSSI     -128

/*---------------------------------------------------------------------------*/

PROCESS(channel_scanner_process, "channel scanner logger");
AUTOSTART_PROCESSES(&channel_scanner_process);

/*---------------------------------------------------------------------------*/

static radio_value_t rssi_buf[NUM_CHANNELS];


/* Specifies the number of rows to print of the RSSI graph */
#define NUM_MAG_BINS        16

/* Each bin is full scale (int8 / number of bins) approximate with 128 as full scale */
#define MAG_BIN_FS          (128)
#define MAG_BIN_THRESH      (MAG_BIN_FS/(NUM_MAG_BINS))

#define RSSI_GRAPH_INDICATOR '*'

static void
print_rssi(int start_chan, int end_chan)
{
    int num_channels = end_chan - start_chan + 1;
    if (num_channels > NUM_CHANNELS ||
        end_chan < start_chan) {
        return;
    }

    char graph_buf[NUM_CHANNELS+10];
    memset(graph_buf, 0, sizeof(graph_buf));
    memset(graph_buf, '-', num_channels);
    printf("RSSI %s\n", graph_buf);
    /* sweep the magnitude bins 0 to full scale by increments of 128/NUM_MAG_BINS */
    for (int mag_bin = 0 - MAG_BIN_THRESH; mag_bin >= -MAG_BIN_FS; mag_bin -= MAG_BIN_THRESH) {
        /*sweep the rssi bins.*/
        printf("%03d: ", mag_bin);
        for (int rssi_bin = 0; rssi_bin < num_channels; rssi_bin++) {
            /* if value is greater than the mag_bin threshold, print the indicator else print space */
            graph_buf[rssi_bin] = (rssi_buf[start_chan + rssi_bin] > NO_RSSI && 
                rssi_buf[start_chan + rssi_bin] >= mag_bin) ? RSSI_GRAPH_INDICATOR : ' ';
        }
        /* graph_buf is always terminated */
        printf("%s\n", graph_buf);
    }

    /* print the axis */
    memset(graph_buf, '-', num_channels);
    printf("     %s\n", graph_buf);
    memset(graph_buf, ' ', num_channels -2);
    sprintf(graph_buf + num_channels-2, "%d", end_chan);
    printf("     %d%s\n\n", start_chan, graph_buf);
}

void print_stats()
{
    /*Find and print largest RSSI measurement and channel */
    int max = NO_RSSI;
    int max_chan = -1;
    for (int c = 0; c < NUM_CHANNELS; c++)
    {
        if (rssi_buf[c] > max) {
            max = rssi_buf[c];
            max_chan = c;
        }
    }
    printf("Max RSSI is %d on channel %d at frequency %d kHz\n", 
        max, max_chan, 
        DOT_15_4G_CHAN0_FREQ + max_chan * DOT_15_4G_FREQ_SPACING);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(channel_scanner_process, ev, data)
{
    PROCESS_BEGIN();

    static struct etimer meas_timer;
    static struct etimer scan_timer;
    static bool led_color_toggle;

    leds_off(LEDS_ALL);
    led_color_toggle = false;

    printf("Starting Mesh Channel Scanner DEMO\n");

    /* Scan */
    etimer_set(&scan_timer, CLOCK_SECOND * 8);

    while (1) {
        PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_TIMER &&
            etimer_expired(&scan_timer));
        
        etimer_reset(&scan_timer);

        static clock_time_t time;
        time = clock_time();
        printf("------ scanning %d channels, start time %lu ---- \n", NUM_CHANNELS, time / CLOCK_SECOND);

        /* Reset the measurement result buffer */
        memset(rssi_buf, NO_RSSI, sizeof(rssi_buf));

        /* Wait some time between setting the channel and grabbing the RSSI to 
         * avoid failing the RSSI read.
         * Around 4 ticks is required (experimentally determined).
         */
        etimer_set(&meas_timer, 4);

        static int measurement_channel = DOT_15_4G_CHAN_MIN;
        for (measurement_channel = DOT_15_4G_CHAN_MIN; measurement_channel <= DOT_15_4G_CHAN_MAX; measurement_channel++) {

            radio_result_t rv = RADIO_RESULT_OK;
            radio_value_t value;

            if (led_color_toggle) {
                leds_single_toggle(LEDS_RED);
            }
            else {
                leds_single_toggle(LEDS_GREEN);
            }
            led_color_toggle = !led_color_toggle;

            PRINTF("[%d] ... ", measurement_channel);
            /* Select the measurement channel */
            rv = NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, measurement_channel);
            if (rv == RADIO_RESULT_OK) {
                /* Need some delay after a channel change in order to get an RSSI reading.
                 * Without delay the radio RX is not re-enabled in time, and the RSSI
                 * RX-enabler is activated which seems to get in the way.  Even though
                 * The RX status is ACTIVE before making the RSSI request, it comes back
                 * with an error. So here we are.
                 */
                PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_TIMER &&
                    etimer_expired(&meas_timer));
                etimer_reset(&meas_timer);

                rv = NETSTACK_RADIO.get_value(RADIO_PARAM_RSSI, &value);
                if (rv == RADIO_RESULT_OK) {
                    PRINTF("noise RSSI: %d\n", (int)value);
                    rssi_buf[measurement_channel - DOT_15_4G_CHAN_MIN] = value;
                }
                else {
                    printf("RSSI on channel %d failed: %d\n", measurement_channel, rv);
                }
            }else {
                printf("channel %d change failed: %d\n", measurement_channel, rv);
            }
        }

        leds_off(LEDS_ALL);
        /* stop the timer */
        etimer_stop(&meas_timer);
        printf("------ scan cycle duration %lu s, per channel: %lu ms ----\n",
            (clock_time() - time) / CLOCK_SECOND,
            ((clock_time() - time) * 1000) / CLOCK_SECOND / NUM_CHANNELS);

#if (NUM_CHANNELS > 80) 
            print_rssi(DOT_15_4G_CHAN_MIN, DOT_15_4G_CHAN_MAX / 2);
            print_rssi(DOT_15_4G_CHAN_MAX / 2 + 1, DOT_15_4G_CHAN_MAX);
#else
            print_rssi(DOT_15_4G_CHAN_MIN, DOT_15_4G_CHAN_MAX);
#endif        
            print_stats();
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
