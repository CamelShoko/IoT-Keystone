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
*         Contiki driver for LoRaMac.
*
*
* \author Evan Ross <evan@thisisiot.io>
*/

#ifndef __LORAMACCONTIKI_H__
#define __LORAMACCONTIKI_H__

/*!
 * In ACK send mode, this is the number of retries for the message.
 */
#ifdef LORA_CONF_NUM_TX_ACK_RETRY
#define LORA_NUM_TX_ACK_RETRY   LORA_CONF_NUM_TX_ACK_RETRY
#else
#define LORA_NUM_TX_ACK_RETRY       3
#endif


typedef enum 
{
    SendStatus_SENT = 0,
    SendStatus_PENDING,
    SendStatus_QUEUE_FULL,
    SendStatus_TOO_LARGE,
    SendStatus_FAILED
} SendStatus_t;

typedef enum
{
    EventType_NONE,
    EventType_JOINED,
    EventType_MAX
} EventType_t;

/* Event posted to user process when join is complete. */
extern process_event_t lora_op_complete_event;

/*!
 * Platform radio driver must poll this process from its IRQ context
 * to trigger IRQ handling.
 */
PROCESS_NAME(loramac_process);

/*!
 * Start the LoRaMac stack.
 *
 * Supply the device's AppEui (8 bytes) and AppKey (16 bytes).
 * NULL uses defaults setup in lora-conf.h
 *
 * Causes a network join to occur.  Once joined, the stack is ready to accept
 * data for transmission.
 *
 * A user process, if specified, is notified of a successful join by
 * posting lora_op_complete_event to it.
 */
void LoRaMacContiki_start(void* process, const uint8_t* user_app_eui, const uint8_t* user_app_key);


/* Platform-supplied functions */

/* Get the platform battery level */
extern uint8_t BoardGetBatteryLevel(void);


#endif
