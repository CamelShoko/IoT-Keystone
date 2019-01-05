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
*         Contiki driver for LoRaMac.
*
*
* \author Evan Ross <evan@thisisiot.io>
*
* Ported from LoRaMac-Node  apps/LoRaMac/classA/NucleoL152/main.c
*/

/*!
* \file      main.c
*
* \brief     LoRaMac classA device implementation
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

/*! \file classA/NucleoL152/main.c */

#include <stdio.h>
#include <string.h>
#include "contiki.h"
#include "net\linkaddr.h"
#include "dev\leds.h"
#include "lora-utilities.h"
#include "LoRaMac.h"
#include "lora-conf.h"
#include "LoRaMacContiki.h"
#include "NvmCtxMgmt.h"

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU915 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_US915

#endif

/*---------------------------------------------------------------------------*/
/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE "LoRaContik"
#define LOG_LEVEL LOG_LEVEL_LORA
/*---------------------------------------------------------------------------*/

/*!
* Defines the application data transmission duty cycle. 5s, value in [ms].
*/
#define APP_TX_DUTYCYCLE                            5000

/*!
* Defines a random delay for application data transmission duty cycle. 1s,
* value in [ms].
*/
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
* Default datarate
*/
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
* LoRaWAN confirmed messages
*/
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
* LoRaWAN Adaptive Data Rate
*
* \remark Please note that when ADR is enabled the end-device should be static
*/
#define LORAWAN_ADR_ON                              1

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
* LoRaWAN ETSI duty cycle control enable/disable
*
* \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
*/
#define LORAWAN_DUTYCYCLE_ON                        true

#endif

/*!
* LoRaWAN application port
*/
#define LORAWAN_APP_PORT                            2

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t JoinEui[] = LORAWAN_JOIN_EUI;
static uint8_t AppKey[] = LORAWAN_APP_KEY;
static uint8_t NwkKey[] = LORAWAN_NWK_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t FNwkSIntKey[] = LORAWAN_F_NWK_S_INT_KEY;
static uint8_t SNwkSIntKey[] = LORAWAN_S_NWK_S_INT_KEY;
static uint8_t NwkSEncKey[] = LORAWAN_NWK_S_ENC_KEY;
static uint8_t AppSKey[] = LORAWAN_APP_S_KEY;

/*!
* Device address
*/
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
* Application port
*/
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
* User application data size
*/
static uint8_t AppDataSize = 1;
static uint8_t AppDataSizeBackup = 1;

/*!
* User application data buffer size
*/
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
* User application data
*/
static uint8_t AppDataBuffer[LORAWAN_APP_DATA_MAX_SIZE];

/*!
* Indicates if the node is sending confirmed or unconfirmed messages
*/
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
* Defines the application data transmission duty cycle
*/
static uint32_t TxDutyCycleTime;

/*!
* Timer to handle the application data transmission duty cycle
*/
static TimerEvent_t TxNextPacketTimer;

/*!
* Specifies the state of the application LED
*/
static bool AppLedStateOn = false;

/*!
* Timer to handle the state of LED1
*/
static TimerEvent_t Led1Timer;

/*!
* Timer to handle the state of LED2
*/
static TimerEvent_t Led2Timer;

/*!
* Indicates if a new packet can be sent
*/
static bool NextTx = true;

/*!
* Indicates if LoRaMacProcess call is pending.
*
* \warning If variable is equal to 0 then the MCU can be set in low power mode
*/
static uint8_t IsMacProcessPending = 0;

/*!
* Device states
*/
static enum eDeviceState
{
    DEVICE_STATE_RESTORE,
    DEVICE_STATE_START,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
* LoRaWAN compliance tests support data
*/
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*!
*
*/
typedef enum
{
    LORAMAC_HANDLER_UNCONFIRMED_MSG = 0,
    LORAMAC_HANDLER_CONFIRMED_MSG = !LORAMAC_HANDLER_UNCONFIRMED_MSG
}LoRaMacHandlerMsgTypes_t;

/*!
* Application data structure
*/
typedef struct LoRaMacHandlerAppData_s
{
    LoRaMacHandlerMsgTypes_t MsgType;
    uint8_t Port;
    uint8_t BufferSize;
    uint8_t *Buffer;
}LoRaMacHandlerAppData_t;

LoRaMacHandlerAppData_t AppData =
{
    .MsgType = LORAMAC_HANDLER_UNCONFIRMED_MSG,
    .Buffer = NULL,
    .BufferSize = 0,
    .Port = 0
};

                    /*!
                    * MAC status strings
                    */
const char* MacStatusStrings[] =
{
    "OK",                            // LORAMAC_STATUS_OK
    "Busy",                          // LORAMAC_STATUS_BUSY
    "Service unknown",               // LORAMAC_STATUS_SERVICE_UNKNOWN
    "Parameter invalid",             // LORAMAC_STATUS_PARAMETER_INVALID
    "Frequency invalid",             // LORAMAC_STATUS_FREQUENCY_INVALID
    "Datarate invalid",              // LORAMAC_STATUS_DATARATE_INVALID
    "Frequency or datarate invalid", // LORAMAC_STATUS_FREQ_AND_DR_INVALID
    "No network joined",             // LORAMAC_STATUS_NO_NETWORK_JOINED
    "Length error",                  // LORAMAC_STATUS_LENGTH_ERROR
    "Region not supported",          // LORAMAC_STATUS_REGION_NOT_SUPPORTED
    "Skipped APP data",              // LORAMAC_STATUS_SKIPPED_APP_DATA
    "Duty-cycle restricted",         // LORAMAC_STATUS_DUTYCYCLE_RESTRICTED
    "No channel found",              // LORAMAC_STATUS_NO_CHANNEL_FOUND
    "No free channel found",         // LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND
    "Busy beacon reserved time",     // LORAMAC_STATUS_BUSY_BEACON_RESERVED_TIME
    "Busy ping-slot window time",    // LORAMAC_STATUS_BUSY_PING_SLOT_WINDOW_TIME
    "Busy uplink collision",         // LORAMAC_STATUS_BUSY_UPLINK_COLLISION
    "Crypto error",                  // LORAMAC_STATUS_CRYPTO_ERROR
    "FCnt handler error",            // LORAMAC_STATUS_FCNT_HANDLER_ERROR
    "MAC command error",             // LORAMAC_STATUS_MAC_COMMAD_ERROR
    "ClassB error",                  // LORAMAC_STATUS_CLASS_B_ERROR
    "Confirm queue error",           // LORAMAC_STATUS_CONFIRM_QUEUE_ERROR
    "Unknown error",                 // LORAMAC_STATUS_ERROR
};

/*!
* MAC event info status strings.
*/
const char* EventInfoStatusStrings[] =
{
    "OK",                            // LORAMAC_EVENT_INFO_STATUS_OK
    "Error",                         // LORAMAC_EVENT_INFO_STATUS_ERROR
    "Tx timeout",                    // LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT
    "Rx 1 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT
    "Rx 2 timeout",                  // LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT
    "Rx1 error",                     // LORAMAC_EVENT_INFO_STATUS_RX1_ERROR
    "Rx2 error",                     // LORAMAC_EVENT_INFO_STATUS_RX2_ERROR
    "Join failed",                   // LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL
    "Downlink repeated",             // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED
    "Tx DR payload size error",      // LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR
    "Downlink too many frames loss", // LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS
    "Address fail",                  // LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL
    "MIC fail",                      // LORAMAC_EVENT_INFO_STATUS_MIC_FAIL
    "Multicast fail",                // LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL
    "Beacon locked",                 // LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED
    "Beacon lost",                   // LORAMAC_EVENT_INFO_STATUS_BEACON_LOST
    "Beacon not found"               // LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND
};

/*---------------------------------------------------------------------------*/
PROCESS(loramac_process, "loramac process");
/*---------------------------------------------------------------------------*/

process_event_t lora_op_complete_event;

static struct process* notify_process;

/* Notify the user process of a "lora_op_complete_event */
static void notifyUserEvent(EventType_t type)
{
    /* if notify_process is null nothing happens.
    * once notified, the event is cleared.
    */
    if (notify_process != NULL) {
        process_post(notify_process, lora_op_complete_event, (void*)type);
        notify_process = NULL;
    }
}

/*---------------------------------------------------------------------------*/
/* Populate the stack's DevEui field as requested.
 */
static void BoardGetUniqueId(uint8_t id[sizeof(DevEui)])
{
    /* Use Contiki's globally-defined linkaddr structure 
     * which has a default size of 8 bytes.
     */
    memcpy(id, &linkaddr_node_addr, sizeof(DevEui));
}


/*---------------------------------------------------------------------------*/

/*!
* Prints the provided buffer in HEX
*
* \param buffer Buffer to be printed
* \param size   Buffer size to be printed
*/
void PrintHexBuffer(uint8_t *buffer, uint8_t size)
{
    uint8_t newline = 0;

    for (uint8_t i = 0; i < size; i++)
    {
        if (newline != 0)
        {
            printf("\n");
            newline = 0;
        }

        printf("%02X ", buffer[i]);

        if (((i + 1) % 16) == 0)
        {
            newline = 1;
        }
    }
    printf("\n");
}

/*!
* Executes the network Join request
*/
static void JoinNetwork(void)
{
    LoRaMacStatus_t status;
    MlmeReq_t mlmeReq;
    mlmeReq.Type = MLME_JOIN;
    mlmeReq.Req.Join.DevEui = DevEui;
    mlmeReq.Req.Join.JoinEui = JoinEui;
    mlmeReq.Req.Join.Datarate = LORAWAN_DEFAULT_DATARATE;

    // Starts the join procedure
    status = LoRaMacMlmeRequest(&mlmeReq);
    LOG_INFO("MLME-Request - MLME_JOIN - Status: %s\n", MacStatusStrings[status]);

    if (status == LORAMAC_STATUS_OK)
    {
        LOG_INFO("Joining\n");
        DeviceState = DEVICE_STATE_SLEEP;
    }
    else
    {
        DeviceState = DEVICE_STATE_CYCLE;
    }
}

/*!
* \brief   Prepares the payload of the frame
*/
static void PrepareTxFrame(uint8_t port)
{
    switch (port)
    {
    case 2:
    {
        AppDataSizeBackup = AppDataSize = 1;
        AppDataBuffer[0] = AppLedStateOn;
    }
    break;
    case 224:
        if (ComplianceTest.LinkCheck == true)
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppDataBuffer[0] = 5;
            AppDataBuffer[1] = ComplianceTest.DemodMargin;
            AppDataBuffer[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch (ComplianceTest.State)
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppDataBuffer[0] = ComplianceTest.DownLinkCounter >> 8;
                AppDataBuffer[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
* \brief   Prepares the payload of the frame
*
* \retval  [0: frame could be send, 1: error]
*/
static bool SendFrame(void)
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if (LoRaMacQueryTxPossible(AppDataSize, &txInfo) != LORAMAC_STATUS_OK)
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if (IsTxConfirmed == false)
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppDataBuffer;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppDataBuffer;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    // Update global variable
    AppData.MsgType = (mcpsReq.Type == MCPS_CONFIRMED) ? LORAMAC_HANDLER_CONFIRMED_MSG : LORAMAC_HANDLER_UNCONFIRMED_MSG;
    AppData.Port = mcpsReq.Req.Unconfirmed.fPort;
    AppData.Buffer = mcpsReq.Req.Unconfirmed.fBuffer;
    AppData.BufferSize = mcpsReq.Req.Unconfirmed.fBufferSize;

    LoRaMacStatus_t status;
    status = LoRaMacMcpsRequest(&mcpsReq);
    LOG_INFO("MCPS-Request type:%d port:%d size:%d rate:%d. Status : %s\n",
        AppData.MsgType, AppData.Port, AppData.BufferSize, mcpsReq.Req.Unconfirmed.Datarate,
        MacStatusStrings[status]);

    if (status == LORAMAC_STATUS_OK)
    {
        return false;
    }
    return true;
}

/*!
* \brief Function executed on TxNextPacket Timeout event
*/
static void OnTxNextPacketTimerEvent(void)
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop(&TxNextPacketTimer);

    mibReq.Type = MIB_NETWORK_ACTIVATION;
    status = LoRaMacMibGetRequestConfirm(&mibReq);

    if (status == LORAMAC_STATUS_OK)
    {
        if (mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE)
        {
            // Network not joined yet. Try to join again
            JoinNetwork();
        }
        else
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
    }

    process_post(&loramac_process, PROCESS_EVENT_POLL, 0);
}

/*!
* \brief Function executed on Led 1 Timeout event
*/
static void OnLed1TimerEvent(void)
{
    TimerStop(&Led1Timer);
    // Switch LED 1 OFF
    leds_single_off(LEDS_GREEN);
}

/*!
* \brief Function executed on Led 2 Timeout event
*/
static void OnLed2TimerEvent(void)
{
    TimerStop(&Led2Timer);
    // Switch LED 2 OFF
    leds_single_off(LEDS_RED);
}

/*!
* \brief   MCPS-Confirm event function
*
* \param   [IN] mcpsConfirm - Pointer to the confirm structure,
*               containing confirm attributes.
*/
static void McpsConfirm(McpsConfirm_t *mcpsConfirm)
{
    LOG_INFO("MCPS-Confirm. Status : %s\n", EventInfoStatusStrings[mcpsConfirm->Status]);
    if (mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK)
    {
    }
    else
    {
        switch (mcpsConfirm->McpsRequest)
        {
        case MCPS_UNCONFIRMED:
        {
            // Check Datarate
            // Check TxPower
            break;
        }
        case MCPS_CONFIRMED:
        {
            // Check Datarate
            // Check TxPower
            // Check AckReceived
            // Check NbTrials
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        default:
            break;
        }

        // Switch LED 1 ON
        leds_single_on(LEDS_GREEN);
        TimerStart(&Led1Timer);
    }
    MibRequestConfirm_t mibGet;
    MibRequestConfirm_t mibReq;

    mibReq.Type = MIB_DEVICE_CLASS;
    LoRaMacMibGetRequestConfirm(&mibReq);

    LOG_INFO("UPLINK FRAME %lu Class:%c TX port:%d\n",
        mcpsConfirm->UpLinkCounter, 
        "ABC"[mibReq.Param.Class],
        AppData.Port);

    if (AppData.BufferSize != 0)
    {
        if (AppData.MsgType == LORAMAC_HANDLER_CONFIRMED_MSG)
        {
            LOG_INFO("TX CONFIRMED - %s | ", (mcpsConfirm->AckReceived != 0) ? "ACK" : "NACK");
        }
        else
        {
            LOG_INFO("TX UNCONFIRMED | ");
        }
        PrintHexBuffer(AppData.Buffer, AppData.BufferSize);
    }

    LOG_INFO("DATA RATE   : DR_%d\n", mcpsConfirm->Datarate);

    mibGet.Type = MIB_CHANNELS;
    if (LoRaMacMibGetRequestConfirm(&mibGet) == LORAMAC_STATUS_OK)
    {
        LOG_INFO("U/L FREQ    : %lu\n", mibGet.Param.ChannelList[mcpsConfirm->Channel].Frequency);
    }

    LOG_INFO("TX POWER    : %d\n", mcpsConfirm->TxPower);

    mibGet.Type = MIB_CHANNELS_MASK;
    if (LoRaMacMibGetRequestConfirm(&mibGet) == LORAMAC_STATUS_OK)
    {
        LOG_INFO("CHANNEL MASK: ");
#if defined( REGION_AS923 ) || defined( REGION_CN779 ) || \
    defined( REGION_EU868 ) || defined( REGION_IN865 ) || \
    defined( REGION_KR920 ) || defined( REGION_EU433 ) || \
    defined( REGION_RU864 )

        for (uint8_t i = 0; i < 1; i++)

#elif defined( REGION_AU915 ) || defined( REGION_US915 ) || defined( REGION_CN470 )

        for (uint8_t i = 0; i < 5; i++)
#else

#error "Please define a region in the compiler options."

#endif
        {
            printf("%04X ", mibGet.Param.ChannelsMask[i]);
        }
        printf("\n");
    }
}

/*!
* \brief   MCPS-Indication event function
*
* \param   [IN] mcpsIndication - Pointer to the indication structure,
*               containing indication attributes.
*/
static void McpsIndication(McpsIndication_t *mcpsIndication)
{
    LOG_INFO("MCPS-Indication. Status : %s\n", EventInfoStatusStrings[mcpsIndication->Status]);
    if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK)
    {
        return;
    }

    switch (mcpsIndication->McpsIndication)
    {
    case MCPS_UNCONFIRMED:
    {
        break;
    }
    case MCPS_CONFIRMED:
    {
        break;
    }
    case MCPS_PROPRIETARY:
    {
        break;
    }
    case MCPS_MULTICAST:
    {
        break;
    }
    default:
        break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    if (mcpsIndication->FramePending == true)
    {
        // The server signals that it has pending data to be sent.
        // We schedule an uplink as soon as possible to flush the server.
        OnTxNextPacketTimerEvent();
    }
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if (ComplianceTest.Running == true)
    {
        ComplianceTest.DownLinkCounter++;
    }

    if (mcpsIndication->RxData == true)
    {
        switch (mcpsIndication->Port)
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if (mcpsIndication->BufferSize == 1)
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
            }
            break;
        case 224:
            if (ComplianceTest.Running == false)
            {
                // Check compliance test enable command (i)
                if ((mcpsIndication->BufferSize == 4) &&
                    (mcpsIndication->Buffer[0] == 0x01) &&
                    (mcpsIndication->Buffer[1] == 0x01) &&
                    (mcpsIndication->Buffer[2] == 0x01) &&
                    (mcpsIndication->Buffer[3] == 0x01))
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSizeBackup = AppDataSize;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm(&mibReq);

#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn(false);
#endif
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch (ComplianceTest.State)
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = AppDataSizeBackup;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm(&mibReq);
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#endif
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppDataBuffer[0] = 4;
                    for (uint8_t i = 1; i < MIN(AppDataSize, LORAWAN_APP_DATA_MAX_SIZE); i++)
                    {
                        AppDataBuffer[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                {
                    MlmeReq_t mlmeReq;
                    mlmeReq.Type = MLME_LINK_CHECK;
                    LoRaMacStatus_t status = LoRaMacMlmeRequest(&mlmeReq);
                    LOG_INFO("MLME-Request - MLME_LINK_CHECK. Status : %s\n", MacStatusStrings[status]);
                }
                break;
                case 6: // (ix)
                {
                    // Disable TestMode and revert back to normal operation
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = AppDataSizeBackup;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm(&mibReq);
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#endif

                    JoinNetwork();
                }
                break;
                case 7: // (x)
                {
                    if (mcpsIndication->BufferSize == 3)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_TXCW;
                        mlmeReq.Req.TxCw.Timeout = (uint16_t)((mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2]);
                        LoRaMacStatus_t status = LoRaMacMlmeRequest(&mlmeReq);
                        LOG_INFO("MLME-Request - MLME_TXCW. Status : %s\n", MacStatusStrings[status]);
                    }
                    else if (mcpsIndication->BufferSize == 7)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_TXCW_1;
                        mlmeReq.Req.TxCw.Timeout = (uint16_t)((mcpsIndication->Buffer[1] << 8) | mcpsIndication->Buffer[2]);
                        mlmeReq.Req.TxCw.Frequency = (uint32_t)((mcpsIndication->Buffer[3] << 16) | (mcpsIndication->Buffer[4] << 8) | mcpsIndication->Buffer[5]) * 100;
                        mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                        LoRaMacStatus_t status = LoRaMacMlmeRequest(&mlmeReq);
                        LOG_INFO("MLME-Request - MLME_TXCW1. Status : %s\n", MacStatusStrings[status]);
                    }
                    ComplianceTest.State = 1;
                }
                break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
    leds_single_on(LEDS_RED);
    TimerStart(&Led2Timer);

    const char *slotStrings[] = { "1", "2", "C", "Ping-Slot", "Multicast Ping-Slot" };
    int32_t snr = 0;
    if (mcpsIndication->Snr & 0x80) // The SNR sign bit is 1
    {
        // Invert and divide by 4
        snr = ((~mcpsIndication->Snr + 1) & 0xFF) >> 2;
        snr = -snr;
    }
    else
    {
        // Divide by 4
        snr = (mcpsIndication->Snr & 0xFF) >> 2;
    }

    LOG_INFO("DOWNLINK FRAME %lu RX WINDOW : %s RX PORT : %d\n", 
        mcpsIndication->DownLinkCounter,
        slotStrings[mcpsIndication->RxSlot],
        mcpsIndication->Port);


    if (mcpsIndication->BufferSize != 0)
    {
        LOG_INFO("RX DATA : ");
        PrintHexBuffer(mcpsIndication->Buffer, mcpsIndication->BufferSize);
    }

    LOG_INFO("RX DATA RATE : DR_%d RSSI : %d SNR : %ld\r\n",
        mcpsIndication->RxDatarate,
        mcpsIndication->Rssi,
        snr);
}

/*!
* \brief   MLME-Confirm event function
*
* \param   [IN] mlmeConfirm - Pointer to the confirm structure,
*               containing confirm attributes.
*/
static void MlmeConfirm(MlmeConfirm_t *mlmeConfirm)
{
    LOG_INFO("MLME-Confirm. Status : %s\n", EventInfoStatusStrings[mlmeConfirm->Status]);
    if (mlmeConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK)
    {
    }
    switch (mlmeConfirm->MlmeRequest)
    {
    case MLME_JOIN:
    {
        if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
        {
            MibRequestConfirm_t mibGet;
            LOG_INFO("Joined OTAA.\n");

            mibGet.Type = MIB_DEV_ADDR;
            LoRaMacMibGetRequestConfirm(&mibGet);
            LOG_INFO("Device addr     : %08lX\n", mibGet.Param.DevAddr);

            mibGet.Type = MIB_CHANNELS_DATARATE;
            LoRaMacMibGetRequestConfirm(&mibGet);
            LOG_INFO("Data rate   : DR_%d\n", mibGet.Param.ChannelsDatarate);
            // Status is OK, node has joined the network
            DeviceState = DEVICE_STATE_SEND;

            /* Notify user process of successful join */
            notifyUserEvent(EventType_JOINED);
        }
        else
        {
            // Join was not successful. Try to join again
            JoinNetwork();
        }

        process_post(&loramac_process, PROCESS_EVENT_POLL, 0);

        break;
    }
    case MLME_LINK_CHECK:
    {
        if (mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK)
        {
            // Check DemodMargin
            // Check NbGateways
            if (ComplianceTest.Running == true)
            {
                ComplianceTest.LinkCheck = true;
                ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
            }
        }
        break;
    }
    default:
        break;
    }
}

/*!
* \brief   MLME-Indication event function
*
* \param   [IN] mlmeIndication - Pointer to the indication structure.
*/
static void MlmeIndication(MlmeIndication_t *mlmeIndication)
{
    if (mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED)
    {
        LOG_INFO("MLME-Indication. Status : %s\n", EventInfoStatusStrings[mlmeIndication->Status]);
    }
    if (mlmeIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK)
    {
    }
    switch (mlmeIndication->MlmeIndication)
    {
    case MLME_SCHEDULE_UPLINK:
    {// The MAC signals that we shall provide an uplink as soon as possible
        OnTxNextPacketTimerEvent();
        break;
    }
    default:
        break;
    }
}

void OnMacProcessNotify(void)
{
    IsMacProcessPending = 1;
    process_post(&loramac_process, PROCESS_EVENT_POLL, 0);
}

#define xstr(s) str(s)
#define str(s) #s

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(loramac_process, ev, data)
{
    static LoRaMacPrimitives_t macPrimitives;
    static LoRaMacCallback_t macCallbacks;
    static MibRequestConfirm_t mibReq;
    static LoRaMacStatus_t status;

    PROCESS_BEGIN();

    leds_off(LEDS_ALL);

    LOG_INFO("LoRaMacContiki starting for region: " xstr(ACTIVE_REGION) ".\n");

    lora_op_complete_event = process_alloc_event();

    macPrimitives.MacMcpsConfirm = McpsConfirm;
    macPrimitives.MacMcpsIndication = McpsIndication;
    macPrimitives.MacMlmeConfirm = MlmeConfirm;
    macPrimitives.MacMlmeIndication = MlmeIndication;
    macCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
    macCallbacks.GetTemperatureLevel = NULL;
    macCallbacks.NvmContextChange = NvmCtxMgmtEvent;
    macCallbacks.MacProcessNotify = OnMacProcessNotify;

    LoRaMacInitialization(&macPrimitives, &macCallbacks, ACTIVE_REGION);

    DeviceState = DEVICE_STATE_RESTORE;

    while (1)
    {
        // Process Radio IRQ
        if (Radio.IrqProcess != NULL)
        {
            Radio.IrqProcess();
        }
        // Processes the LoRaMac events
        LoRaMacProcess();

        if (DeviceState == DEVICE_STATE_RESTORE)
        {
            // Try to restore from NVM and query the mac if possible.
            if (NvmCtxMgmtRestore() == NVMCTXMGMT_STATUS_SUCCESS)
            {
                LOG_INFO("###### ===== CTXS RESTORED ==== ######\n");
            }
            else
            {
                mibReq.Type = MIB_APP_KEY;
                mibReq.Param.AppKey = AppKey;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_NWK_KEY;
                mibReq.Param.NwkKey = NwkKey;
                LoRaMacMibSetRequestConfirm(&mibReq);

                // Initialize LoRaMac device unique ID if not already defined in Commissioning.h
                if ((DevEui[0] == 0) && (DevEui[1] == 0) &&
                    (DevEui[2] == 0) && (DevEui[3] == 0) &&
                    (DevEui[4] == 0) && (DevEui[5] == 0) &&
                    (DevEui[6] == 0) && (DevEui[7] == 0))
                {
                    BoardGetUniqueId(DevEui);
                }

#if( OVER_THE_AIR_ACTIVATION == 0 )
                // Choose a random device address if not already defined in Commissioning.h
                if (DevAddr == 0)
                {
                    // Random seed initialization
                    srand1(BoardGetRandomSeed());

                    // Choose a random device address
                    DevAddr = randr(0, 0x01FFFFFF);
                }

                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_F_NWK_S_INT_KEY;
                mibReq.Param.FNwkSIntKey = FNwkSIntKey;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_S_NWK_S_INT_KEY;
                mibReq.Param.SNwkSIntKey = SNwkSIntKey;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_NWK_S_ENC_KEY;
                mibReq.Param.NwkSEncKey = NwkSEncKey;
                LoRaMacMibSetRequestConfirm(&mibReq);

                mibReq.Type = MIB_APP_S_KEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm(&mibReq);
#endif
            }
            DeviceState = DEVICE_STATE_START;
        }
        else if (DeviceState == DEVICE_STATE_START)
        {
            TimerInit(&TxNextPacketTimer, OnTxNextPacketTimerEvent);

            TimerInit(&Led1Timer, OnLed1TimerEvent);
            TimerSetValue(&Led1Timer, 25);

            TimerInit(&Led2Timer, OnLed2TimerEvent);
            TimerSetValue(&Led2Timer, 25);

            mibReq.Type = MIB_PUBLIC_NETWORK;
            mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
            LoRaMacMibSetRequestConfirm(&mibReq);

            mibReq.Type = MIB_ADR;
            mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
            LoRaMacMibSetRequestConfirm(&mibReq);

#if defined( REGION_EU868 )
            LoRaMacTestSetDutyCycleOn(LORAWAN_DUTYCYCLE_ON);
#endif
            mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
            mibReq.Param.SystemMaxRxError = 20;
            LoRaMacMibSetRequestConfirm(&mibReq);

            LoRaMacStart();

            mibReq.Type = MIB_NETWORK_ACTIVATION;
            status = LoRaMacMibGetRequestConfirm(&mibReq);

            if (status == LORAMAC_STATUS_OK)
            {
                if (mibReq.Param.NetworkActivation == ACTIVATION_TYPE_NONE)
                {
                    DeviceState = DEVICE_STATE_JOIN;
                }
                else
                {
                    DeviceState = DEVICE_STATE_SEND;
                    NextTx = true;
                }
            }
        }
        else if (DeviceState == DEVICE_STATE_JOIN)
        {
            LOG_INFO("DevEui      : %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n", 
                DevEui[0], DevEui[1], DevEui[2], DevEui[3],
                DevEui[4], DevEui[5], DevEui[6], DevEui[7]);

            LOG_INFO("AppEui      : %02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X\n", 
                JoinEui[0], JoinEui[1], JoinEui[2], JoinEui[3],
                JoinEui[4], JoinEui[5], JoinEui[6], JoinEui[7]);

            LOG_INFO("AppKey      : %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                NwkKey[0], NwkKey[1], NwkKey[2], NwkKey[3], NwkKey[4], NwkKey[5], NwkKey[6], NwkKey[7],
                NwkKey[8], NwkKey[9], NwkKey[10], NwkKey[11], NwkKey[12], NwkKey[13], NwkKey[14], NwkKey[15]);

#if( OVER_THE_AIR_ACTIVATION == 0 )
            LOG_INFO("###### ===== JOINED ==== ######\r\n");
            LOG_INFO("\r\nABP\r\n\r\n");
            LOG_INFO("DevAddr     : %08lX\r\n", DevAddr);
            LOG_INFO("NwkSKey     : %02X", FNwkSIntKey[0]);
            for (int i = 1; i < 16; i++)
            {
                printf(" %02X", FNwkSIntKey[i]);
            }
            printf("\r\n");
            LOG_INFO("AppSKey     : %02X", AppSKey[0]);
            for (int i = 1; i < 16; i++)
            {
                printf(" %02X", AppSKey[i]);
            }
            printf("\n\r\n");

            // Tell the MAC layer which network server version are we connecting too.
            mibReq.Type = MIB_ABP_LORAWAN_VERSION;
            mibReq.Param.AbpLrWanVersion.Value = ABP_ACTIVATION_LRWAN_VERSION;
            LoRaMacMibSetRequestConfirm(&mibReq);

            mibReq.Type = MIB_NETWORK_ACTIVATION;
            mibReq.Param.NetworkActivation = ACTIVATION_TYPE_ABP;
            LoRaMacMibSetRequestConfirm(&mibReq);

            DeviceState = DEVICE_STATE_SEND;
#else
            JoinNetwork();
#endif
        }
        else if (DeviceState == DEVICE_STATE_SEND)
        {
            if (NextTx == true)
            {
                PrepareTxFrame(AppPort);

                NextTx = SendFrame();
            }
            DeviceState = DEVICE_STATE_CYCLE;
        }
        else if (DeviceState == DEVICE_STATE_CYCLE)
        {
            DeviceState = DEVICE_STATE_SLEEP;
            if (ComplianceTest.Running == true)
            {
                // Schedule next packet transmission
                TxDutyCycleTime = 5000; // 5000 ms
            }
            else
            {
                // Schedule next packet transmission
                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
            }

            // Schedule next packet transmission
            TimerSetValue(&TxNextPacketTimer, TxDutyCycleTime);
            TimerStart(&TxNextPacketTimer);
        }
        else if (DeviceState == DEVICE_STATE_SLEEP)
        {
            if (NvmCtxMgmtStore() == NVMCTXMGMT_STATUS_SUCCESS)
            {
                LOG_INFO("###### ===== CTXS STORED ==== ######\n");
            }

            //CRITICAL_SECTION_BEGIN();
            if (IsMacProcessPending == 1)
            {
                // Clear flag and prevent MCU to go into low power modes.
                IsMacProcessPending = 0;
            }
            else
            {
                // The MCU wakes up through events
                //BoardLowPowerHandler();
                PROCESS_WAIT_EVENT();
            }
            //CRITICAL_SECTION_END();
        }
        else
        {
            DeviceState = DEVICE_STATE_START;
        }
    }

    PROCESS_END();
}


/*---------------------------------------------------------------------------*/


void LoRaMacContiki_start(void* process, const uint8_t* user_app_eui, const uint8_t* user_app_key)
{
    /* Note LoRaWAN 1.1.x renames 1.0.x AppEui -> JoinEui and AppKey -> NwkKey */
    if (user_app_eui) {
        memcpy(JoinEui, user_app_eui, sizeof(JoinEui));
    }
    if (user_app_key) {
        memcpy(NwkKey, user_app_key,  sizeof(NwkKey));
    }

    struct process* p = (struct process*)process;
    notify_process = p;
    process_start(&loramac_process, NULL);
}
