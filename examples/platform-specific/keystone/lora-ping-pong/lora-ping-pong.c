/*
* Copyright (c) 2019, THIS. IS. IoT. - https://thisisiot.io
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
*        This application uses the LoRa transceiver (SX1262) on the
*        IoT.Keystone Innovator Board to send and receive PING and PONG messages
*        with another IoT.Keystone board running the same application.
*        The application is auto-configuring, so all that needs to be done is
*        to load the application and turn boards loose.  You should see the
*        green and red LEDs alternately toggle at approximately a 1 Hz rate if
*        nodes are able to communicate with each other.
* 
*        How to use:
*        Both nodes start up as a master and will alternate between 
*        sending a "ping" message and listening for responses.
*        If a node receives a ping message, it turns into a slave node
*        and will respond with a  "pong" for each "ping" it receives.
*   
*        The RED LED is toggled for each message sent (be it a PING or PONG)
*        The GREEN LED is toggled for each message received.
*
*        This application is based on the apps/ping-pong example in the
*        Lora-net/LoRaMac-node GitHub repo.
*   
*
* \author Evan Ross <evan@thisisiot.io>
*/

#include "contiki.h"
#include "sys/log.h"
#include "dev/leds.h"
#include "lib/random.h"
#include "lora/lora-delay.h"
#include "lora/sx126x.h"
#include "lora/lora-utilities.h"

#include <stdlib.h>
#include <string.h>

/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "PingPong"
#define LOG_LEVEL LOG_LEVEL_INFO
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/

/* The processs must be called "loramac_process" as required by
* the radio driver IRQ polling function.
*/
PROCESS(loramac_process, "LoRa ping-pong process");
AUTOSTART_PROCESSES(&loramac_process);

#define RF_FREQUENCY                                915000000 // Hz
//#define RF_FREQUENCY                                903900000 // Hz  First channel of TTN (LoRaWAN channel 8)

#define TX_OUTPUT_POWER                             20        // dBm

#define LORA_BANDWIDTH                              0         
// [0: 125 kHz,   
//  1: 250 kHz,
//  2: 500 kHz,
//  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12] // 7
#define LORA_CODINGRATE                             1         
// [1: 4/5,
//  2: 4/6,
//  3: 4/7,
//  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
}States_t;

const char* state_str[] = {
    "LOWPOWER",
    "RX",
    "RX_TIMEOUT",
    "RX_ERROR",
    "TX",
    "TX_TIMEOUT",
};


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 64 // Define the payload size here
// Random period added to RX receiver on-times so that 
// nodes de-synchronize faster and start ping-ponging
#define RANDOM_RX_WINDOW                            500 // window in ms to select a random RX timeout

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
* Radio events function pointer
*/
static RadioEvents_t RadioEvents;


/*---------------------------------------------------------------------------*/

void OnTxDone(void)
{
    Radio.Sleep();
    State = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep();
    BufferSize = size;
    memcpy(Buffer, payload, BufferSize);
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout(void)
{
    Radio.Sleep();
    State = TX_TIMEOUT;
}

void OnRxTimeout(void)
{
    Radio.Sleep();
    State = RX_TIMEOUT;
}

void OnRxError(RadioIrqErrorCode_t code)
{
    LOG_ERR("RxError code:%d\n", code);
    Radio.Sleep();
    State = RX_ERROR;
}

uint32_t getPktCnt() {
    uint32_t pkt_cnt =
        (uint32_t)Buffer[4] << 0 |
        (uint32_t)Buffer[5] << 8 |
        (uint32_t)Buffer[6] << 16 |
        (uint32_t)Buffer[7] << 24;
    return pkt_cnt;
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(loramac_process, ev, data)
{
    PROCESS_BEGIN();

    leds_single_off(LEDS_GREEN);
    leds_single_off(LEDS_RED);

    LOG_INFO("Starting lora-ping-pong testing app.\n");
#if Board_SX1262_TX_POWER_LIMIT
    if (TX_OUTPUT_POWER > Board_SX1262_TX_POWER_LIMIT) {
        LOG_INFO("TX power (board limited): %d dBm\n", Board_SX1262_TX_POWER_LIMIT);
    }
    else
#endif
    {
        LOG_INFO("TX power: %d dBm\n", TX_OUTPUT_POWER);
    }
    LOG_INFO("Frequency: %d Hz\n", RF_FREQUENCY);
    LOG_INFO("Spreading factor: SF%d\n", LORA_SPREADING_FACTOR);
    LOG_INFO("Bandwidth: %s kHz\n", LoraBWStrings[LORA_BANDWIDTH]);
    LOG_INFO("Coding rate: %s\n", LoraCRStrings[LORA_CODINGRATE]);

    static bool isMaster = true;
    static uint32_t pkt_counter = 0;
    uint8_t i;


    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
        true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
        LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
        LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
        0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

    LOG_INFO("Time on air for %d bytes: %lu ms\n", BUFFER_SIZE, Radio.TimeOnAir(MODEM_LORA, BUFFER_SIZE));

    Radio.Rx(RX_TIMEOUT_VALUE + random_rand() % RANDOM_RX_WINDOW);

    while (1)
    {
        LOG_DBG("loop: state=%s\n", state_str[State]);
        if (State == RX) {
            if (isMaster == true)
            {
                if (BufferSize > 0)
                {
                    if (strncmp((const char*)Buffer, (const char*)PongMsg, 4) == 0)
                    {
                        // Indicates on a LED that the received frame is a PONG
                        leds_single_toggle(LEDS_GREEN);
                        LOG_INFO("Received PONG for packet %lu. size=%d rssi=%d snr=%d.\n",
                            getPktCnt(), BufferSize, RssiValue, SnrValue);

                        // Send the next PING frame
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        pkt_counter++;
                        // fill in the packet counter
                        for (i = 4; i < 8; i++)
                        {
                            Buffer[i] = (pkt_counter >> (8 * (i - 4))) & 0xff;
                        }
                        // We fill the buffer with numbers for the payload
                        for (i = 8; i < BUFFER_SIZE; i++)
                        {
                            Buffer[i] = i - 8;
                        }
                        DelayMs(1);
                        Radio.Send(Buffer, BUFFER_SIZE);
                        LOG_INFO("Sending PING packet %lu.\n",
                            pkt_counter);
                    }
                    else if (strncmp((const char*)Buffer, (const char*)PingMsg, 4) == 0)
                    { // A master already exists then become a slave
                        LOG_INFO("Received PING packet %lu.  Becoming slave.\n", getPktCnt());
                        isMaster = false;
                        leds_single_toggle(LEDS_RED); // Set LED off
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master and start again
                        LOG_INFO("Received UNKNOWN. Resetting to master.\n");
                        isMaster = true;
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }
                }
            }
            else
            {
                if (BufferSize > 0)
                {
                    if (strncmp((const char*)Buffer, (const char*)PingMsg, 4) == 0)
                    {
                        uint32_t pong_pkt_count = getPktCnt();
                        // Indicates on a LED that the received frame is a PING
                        leds_single_toggle(LEDS_GREEN);
                        LOG_INFO("Received PING packet %lu. size=%d rssi=%d snr=%d.\n", 
                            pong_pkt_count, BufferSize, RssiValue, SnrValue);

                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // fill in the packet counter                        
                        for (i = 4; i < 8; i++)
                        {
                            Buffer[i] = (pong_pkt_count >> (8 * (i - 4))) & 0xff;
                        }
                        // We fill the buffer with numbers for the payload
                        for (i = 8; i < BUFFER_SIZE; i++)
                        {
                            Buffer[i] = i - 8;
                        }
                        DelayMs(1);
                        Radio.Send(Buffer, BUFFER_SIZE);
                        LOG_INFO("Sending PONG for packet %lu.\n",
                            pong_pkt_count);
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        LOG_INFO("Received UNKNOWN. Resetting to master.\n");
                        isMaster = true;
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }
                }
            }
            State = LOWPOWER;
        }
        else if (State == TX) {

            // Indicates on a LED that we have sent a PING [Master]
            // Indicates on a LED that we have sent a PONG [Slave]
            leds_single_toggle(LEDS_RED);
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
        }
        else if (State == RX_TIMEOUT || State == RX_ERROR) {
        
            /* Get and report any errors */
            RadioError_t errors = SX126xGetDeviceErrors();
            if (errors.Value != 0) {
                SX126xClearDeviceErrors();
                LOG_ERR("Radio reported errors: 0x%04x\n", errors.Value);
            }

            if (isMaster == true)
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                pkt_counter++;
                // fill in the packet counter
                for (i = 4; i < 8; i++)
                {
                    Buffer[i] = (pkt_counter >> (8 * (i - 4))) & 0xff;
                }
                for (i = 8; i < BUFFER_SIZE; i++)
                {
                    Buffer[i] = i - 8;
                }
                DelayMs(1);
                Radio.Send(Buffer, BUFFER_SIZE);
                LOG_INFO("Sending PING packet %lu.\n",
                    pkt_counter);
            }
            else
            {
                Radio.Rx(RX_TIMEOUT_VALUE);
            }
            State = LOWPOWER;
        }
        else if (State == TX_TIMEOUT) {
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
        }
        else {
            // Wait for any event
            PROCESS_YIELD();
        }

        // Process Radio IRQ
        if (Radio.IrqProcess != NULL)
        {
            Radio.IrqProcess();
        }
    }

    PROCESS_END();
}


