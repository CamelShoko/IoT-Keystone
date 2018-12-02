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
#include "lib/ifft.h"
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
/* FFT processing of audio stream - can perform quick verification of
 * audio processing chain by subjecting to a tone and observing a peak
 * in the console FFT graph that is output periodically.
 */


/* Enable this to replace the audio data stream wioth a fixed test vector 
 * of tones 1, 3, 7 kHz to test the FFT graphing.
 */
//#define TEST_FFT

/* The number of samples sets the number of bins to be one-half the 
 * number of samples because second half of FFT is ignored due
 * to Nyquist theorem (sample rate must be 2* highest audio freq to capture) 
 *    E.g.  Input is known to be sampled at 16 kHz.
 *          Set num samples to 1024, yielding first 512 results as frequency bins 
 *          spread over 8 kHz.  Each bin would cover 8000/512 ~ 16 Hz.
 * 
 * One thing to note is that the resulting 16-bit PCM in the audio stream has to be
 * down-converted to 8-bit for use with the FFT.  We just sign-shift each sample right by 8.
 * The Contiki IFFT performs conversion in place: input in xre and output in xre.
 * xim can be examined for phase, wherase xre contains the magnitude that we are most
 * interested in.
 *
 * Limit this to the width of the console (half of this)
 */
#ifdef TEST_FFT
#define NUM_FFT_SAMPLES    128
#else
#define NUM_FFT_SAMPLES    128
#endif

/* Specifies the number of rows to print of the FFT graph */
#define NUM_MAG_BINS        16

#ifdef TEST_FFT
#define SAMPLE_RATE         16
#else
#define SAMPLE_RATE         (AUDIO_SENSOR_SAMP_RATE_KHZ)
#endif

#define NUM_FREQ_BINS       (NUM_FFT_SAMPLES/2)

 /* Each bin is full scale (int8 / number of bins) approximate with 128 as full scale */
#define MAG_BIN_FS          (128)
#define MAG_BIN_THRESH      (MAG_BIN_FS/(NUM_MAG_BINS))

#define FFT_GRAPH_INDICATOR '*'

static int16_t xre[NUM_FFT_SAMPLES];
static int16_t xim[NUM_FFT_SAMPLES];

static int fft_num_buffers_used = 0; /* record how many buffers used to collect samples */



/* Sample data */
/* Use https://octave-online.net */
/*
We can sample up to Fs/2 Hz sinusoids.
Fs = 16000;            % Sampling frequency
T = 1/Fs;             % Sampling period
L = 128;              % Length of signal
t = (0:L-1)*T;        % Time vector
Create 3 test tones at 1 KHz, quarter power at 3 KHz and 0.75 at 7 KHz.
S = 1.0*sin(2*pi*1000*t) + 0.25*sin(2*pi*3000*t) +  0.75*sin(2*pi*7000*t);
Y = fft(S);
P2 = abs(Y); % note that Contiki's ifft does not divide by n (L) so neither do we here
P1 = P2(1:L/2+1);      % frequency bins = half the sample rate
P1(2:end-1) = 2*P1(2:end-1);   % double power as we are taking only half of spectrum

% scale S by 64 to get 8-bit integers out of it:

*/

#ifdef TEST_FFT

/* Expect tone at output xre index:
*    8, 24, 56
*/

static int16_t test_xre[NUM_FFT_SAMPLES] = {

    0, 58, 23, 97, 0, 97, 23, 58, 0, -58, -23, -97,  0, -97, -23, -58,
    0, 58, 23, 97, 0, 97, 23, 58, 0, -58, -23, -97,  0, -97, -23, -58,
    0, 58, 23, 97, 0, 97, 23, 58, 0, -58, -23, -97,  0, -97, -23, -58,
    0, 58, 23, 97, 0, 97, 23, 58, 0, -58, -23, -97,  0, -97, -23, -58,
    0, 58, 23, 97, 0, 97, 23, 58, 0, -58, -23, -97,  0, -97, -23, -58,
    0, 58, 23, 97, 0, 97, 23, 58, 0, -58, -23, -97,  0, -97, -23, -58,
    0, 58, 23, 97, 0, 97, 23, 58, 0, -58, -23, -97,  0, -97, -23, -58,
    0, 58, 23, 97, 0, 97, 23, 58, 0, -58, -23, -97,  0, -97, -23, -58,

};
#endif



static void
compute_and_print_fft()
{
    memset(xim, 0, sizeof(xim));
#ifdef TEST_FFT
    memcpy(xre, test_xre, sizeof(test_xre));
#endif
    
    ifft(xre, xim, NUM_FFT_SAMPLES);

    /* need to post process:
    *   divide by n, then double.
    */
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
        xre[i] /= (NUM_FFT_SAMPLES / 2);
    }

    //printf("[ ");
    for (int i = 0; i < NUM_FREQ_BINS; i++) {
#ifdef TEST_FFT
        xre[i] *= 2;  /* extra *2 because of only 64 mult on input data */
#endif
        //printf("%d ", xre[i]); 
    }
    //printf("]\n");

    /* re-use xim as a print buffer */
    memset(xim, 0, sizeof(xim));
    char* graph_buf = (char*)xim;

    /* Assuming output is 8 bit too, we can take the xre data to be in
     * the range 0 to 127. We can further bin that to 16 bins of 8 magnitude units.
     *    
     */

    printf("Audio stream FFT on %d ms (%d samples @ %d kHz) buffers=%d: \n", NUM_FFT_SAMPLES / SAMPLE_RATE, NUM_FFT_SAMPLES, SAMPLE_RATE, fft_num_buffers_used);
    printf("  %d frequency bins, %d Hz/bin\n", NUM_FREQ_BINS, (SAMPLE_RATE * 1000)/ NUM_FFT_SAMPLES);
    printf("  %d magnitude bins, %d dB/bin\n", NUM_MAG_BINS, 0 /* don't know proper way to show this */);
    memset(graph_buf, '-', NUM_FREQ_BINS);
    printf("     %s\n", graph_buf);
    /* sweep the magnitude bins 0 to full scale by increments of 128/NUM_MAG_BINS */
    for (int mag_bin = MAG_BIN_FS - MAG_BIN_THRESH; mag_bin > 0 ; mag_bin -= MAG_BIN_THRESH) {
        /*sweep the frequency bins.*/
        printf("%03d: ", mag_bin);
        for (int freq_bin = 0; freq_bin < NUM_FREQ_BINS; freq_bin++) {
            /* if value is greater than the mag_bin threshold, print the indicator else print space */
            graph_buf[freq_bin] = xre[freq_bin] >= mag_bin ? FFT_GRAPH_INDICATOR : ' ';
        }
        /* graph_buf is always terminated */
        printf("%s\n", graph_buf);
           
    }

    /* print the axis */
    memset(graph_buf, '-', NUM_FREQ_BINS);
    printf("     %s\n", graph_buf);
    memset(graph_buf, ' ', NUM_FREQ_BINS-2);
    sprintf(graph_buf + NUM_FREQ_BINS-2, "%d", (SAMPLE_RATE * 1000) / 2);
    printf("     0%s\n", graph_buf);

    fft_num_buffers_used = 0;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(data_logger_process, ev, data)
{
    PROCESS_BEGIN();

    static struct etimer led_timer;
    static struct etimer sample_timer;
    static int fft_sample_count = 0; /* set to non-zero to trigger FFT accumulation */

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

                /*-----AUDIO ---------------------------------*/
                /* Trigger FFT computation */
                fft_sample_count = NUM_FFT_SAMPLES;

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
                        //PRINTF("audio buffer: %d (count=%d)\n", buf->metaData.seqNum, count);
                    }


                    if (fft_sample_count > 0) {
                        /* need to collect NUM_FFT_SAMPLES into xre.
                         * Now this is a bit confusing: we need 8-bit samples stored in 16-bit values,
                         * and audio buffer contains 16-bit PCM but its length is given in bytes. 
                         */
                        fft_num_buffers_used++;

                        /* figure out how many bytes to get from this buffer. */
                        int num_samples = fft_sample_count > AUDIO_SENSOR_RET_BUF_SIZE / 2 ? AUDIO_SENSOR_RET_BUF_SIZE / 2 : fft_sample_count;

                        for (int i = 0; i < num_samples; i++) {
                            int16_t* samples = (int16_t*)buf->buffer; /* cast audio byte buffer as array of 16-bit PCM samples */
                            
                            xre[NUM_FFT_SAMPLES - fft_sample_count] = samples[i] >> 8; /* convert to 8-bit as required by FFT */
                            //printf("%d ", xre[NUM_FFT_SAMPLES - fft_sample_count]);

                            fft_sample_count--;
                        }
                        //printf(" ]\n");

                        if (fft_sample_count == 0) {
                            compute_and_print_fft();
                        }
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
