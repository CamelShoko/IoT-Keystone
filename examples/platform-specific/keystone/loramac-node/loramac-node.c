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
#include "net/mac/lora/LoRaMacContiki.h"

/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "LoRa-Node"
#define LOG_LEVEL LOG_LEVEL_INFO
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(loramac_node_process, "LoRaMac-Node");
AUTOSTART_PROCESSES(&loramac_node_process);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(loramac_node_process, ev, data)
{
    PROCESS_BEGIN();

    static struct etimer et;

    LOG_INFO("Starting LoRaMac-Node Demo\n");
    LoRaMacContiki_start(&loramac_node_process, NULL, NULL);

    LOG_INFO("Waiting for join to complete...\n");

    etimer_set(&et, CLOCK_SECOND * 1);
    while (1) {
        PROCESS_WAIT_EVENT();
        if (ev == lora_op_complete_event) {
            EventType_t type = (EventType_t)data;
            if (type == EventType_JOINED) {
                LOG_INFO("Join complete!\n");
                etimer_stop(&et);
            }
        }
        if (ev == PROCESS_EVENT_TIMER && etimer_expired(&et)) {
            LOG_INFO(".\n");
            etimer_reset(&et);
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
